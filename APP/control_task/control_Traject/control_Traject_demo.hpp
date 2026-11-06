#pragma once

#include "main.h"
#include "control_task.h"
#include "pid_controller.hpp"
#include "chassis_task.h"
#include "topic_pool.h"
#include "topics.hpp"
#include "tool.hpp"
#include <cmath>
#include <cstdint>

// 对照原 TrajectChassis 的 demo，单独命名以便两个头文件同时包含。
// 输入位姿：x/y 为 m，yaw（pub_chassis_cmd::omega_）为 deg。
// 输出速度：vx/vy 为 m/s，omega 为 rad/s，与旧接口一致。
// 轨迹由 q(u) 定义；默认直线 + 最短角度插值，可通过 Set_Trajectory 接入自定义轨迹。
// 速度规划预采样曲线并前后扫描，不把 dq/du 当成常量，也不依赖实际平移。
// 固定容量，无动态分配；建议实例放在静态/全局存储区，避免占用任务栈。
// 每段从静止规划到静止；中途换目标会重新起步，不保证速度连续。
class TrajectChassisDemo {
public:
    struct TrajGeometry {
        chassis_position pose{};       // m, m, deg
        chassis_position dpose_du{};   // 对无量纲 u 的一阶导数
        chassis_position d2pose_du2{}; // 对无量纲 u 的二阶导数
    };

    using PoseFunction = chassis_position (*)(float_t u,
        const chassis_position& start, const chassis_position& goal, const void* context);
    using DerivativeFunction = bool (*)(float_t u,
        const chassis_position& start, const chassis_position& goal, const void* context,
        chassis_position& dpose_du, chassis_position& d2pose_du2);
    using ProgressLimitFunction = float_t (*)(float_t u, const void* context);

    struct TrajectorySource {
        PoseFunction pose = nullptr; // nullptr 使用默认直线轨迹
        DerivativeFunction derivatives = nullptr; // 可选；缺省对 pose 作数值差分
        ProgressLimitFunction speed_limit = nullptr; // 可选局部 u_dot 上限，须为有限正数
        const void* context = nullptr;
        float_t max_u_speed = 1.0f; // 1/s；也约束完全静止的轨迹区间
        float_t max_u_acc = 2.0f;   // 1/s^2
        float_t max_u_dec = 2.0f;
    };

    // 回调须无副作用，相同 u 返回相同结果；context 必须在本段运行期间保持有效且不变。
    // pose(0) 应匹配实测起点，pose(1) 应匹配 Set_Ref 目标（角度按模 360 比较）。
    // 曲线须连续、分段二阶可导且一阶导数连续；折点应平滑或拆成停稳的独立段。
    // 自定义 yaw 可连续展开，不能用“上次调用的 yaw”补分支，也不要对总转角强制最短化。
    // 仅在切换轨迹时调用；Set_Trajectory({}) 恢复默认轨迹。
    // 用法：TrajectorySource src{}; src.pose = MyPose; src.derivatives = MyDerivatives;
    //       demo.Set_Trajectory(src); demo.Set_Ref(goal, limits); 然后周期调用 Run(now)。
    // MyDerivatives 可省略；speed_limit 是局部速度上限，不是必须达到的指定速度。
    void Set_Trajectory(const TrajectorySource& source) {
        trajectory = source;
        Restart();
    }

    // 同一目标重跑、context 内容修改后，都必须显式重建采样规划。
    void Restart() {
        generate_pending = has_ref;
        Param_valid_Flag = true;
        Traj_complete_Flag = false;
        PointTrack_linear_complete_Flag = false;
        PointTrack_omega_complete_Flag = false;
        output_b = {};
    }

    float_t watch_1 = 0.0f; // 角速度前馈 rad/s
    float_t watch_2 = 0.0f; // 角速度反馈 rad/s
    float_t watch_3 = 0.0f; // 当前参考 x
    float_t watch_4 = 0.0f; // 当前参考 y
    float_t watch_5 = 0.0f; // 当前参考 yaw，deg

    bool Traj_complete_Flag = false; // 参考轨迹完成，不代表实际到位
    bool PointTrack_linear_complete_Flag = false;
    bool PointTrack_omega_complete_Flag = false;
    bool Param_valid_Flag = true; // 参数、回调数据或规划无效时置 false，停止输出

    // =====================================================
    //  轨迹运行函数：保留原来的调用接口
    // =====================================================
    void Run(pub_chassis_cmd Now) {
        float_t elapsed = DWT_GetDeltaT(&DWT_CNT);
        if (!clock_ready) {
            clock_ready = true;
            elapsed = 0.0f; // 首次只建立计时基准，不把上电时间作为 dt
        }
        Run(Now, elapsed);
    }

    // 可由调用方提供控制周期，便于观察和仿真；PID 内部仍使用自身 DWT。
    void Run(pub_chassis_cmd Now, float_t dt_in) {
        if (!std::isfinite(Now.linear_x_) || !std::isfinite(Now.linear_y_) ||
            !std::isfinite(Now.omega_) || !std::isfinite(dt_in) || dt_in < 0.0f) {
            output_b = {};
            return;
        }
        now = {Now.linear_x_, Now.linear_y_, Now.omega_};
        dt = dt_in;
        if (!has_ref || !Param_valid_Flag) {
            output_b = {};
            return;
        }
        if (generate_pending) {
            TrajGenerate(); // 使用本次实测位置，而非上一个目标作为起点
            generate_pending = false;
            dt = 0.0f; // 新目标从本次采样时刻开始
        }
        if (!Param_valid_Flag) return;

        Update_u_ref();
        if (!Update_TrajGeometry(u_ref)) return;
        Update_TarjSpeed();
        Update_TrackSpeed_linear(Traj_u_ref);
        Update_TrackSpeed_omega(Traj_u_ref);

        output_w.vx = Traj_wff.vx + track_w.vx;
        output_w.vy = Traj_wff.vy + track_w.vy;
        output_w.w  = Traj_wff.w  + track_w.w;
        Limit_Output(); // 前馈 + 反馈后统一限速

        watch_1 = Traj_wff.w;
        watch_2 = track_w.w;
        watch_3 = Traj_u_ref.x;
        watch_4 = Traj_u_ref.y;
        watch_5 = Traj_u_ref.yaw;

        const float_t cos_yaw = cosf(now.yaw * kDegToRad);
        const float_t sin_yaw = sinf(now.yaw * kDegToRad);
        output_b.vx =  output_w.vx * cos_yaw + output_w.vy * sin_yaw;
        output_b.vy = -output_w.vx * sin_yaw + output_w.vy * cos_yaw;
        output_b.w  = output_w.w;
    }

    // =====================================================
    //  返回最终速度参数 / 参考进度
    // =====================================================
    pub_chassis_cmd Get_output_b() const {
        return {output_b.vx, output_b.vy, output_b.w};
    }
    float_t Get_u_ref() const { return u_ref; }
    float_t Get_u_dot() const { return u_dot; }
    chassis_position Get_reference() const { return Traj_u_ref; }
    chassis_speed Get_feedforward_w() const { return Traj_wff; }

    // =====================================================
    //  设置目标值
    //  speed_data 沿用原结构；额外的角加/减速度单位均为 rad/s^2。
    //  速度、加减速度参数均须为有限正数。
    //  相同目标和参数重复下发不会重新起步；参数变化则重新规划。
    // =====================================================
    void Set_Ref(pub_chassis_cmd Ref, speed_data speed_In,
                 float_t Acc_omega = 3.14f, float_t Dec_omega = 3.14f) {
        const bool valid = std::isfinite(Ref.linear_x_) &&
            std::isfinite(Ref.linear_y_) && std::isfinite(Ref.omega_) &&
            PositiveFinite(speed_In.Acc_linear) && PositiveFinite(speed_In.Dec_linear) &&
            PositiveFinite(speed_In.v_Max) && PositiveFinite(speed_In.w_Max) &&
            PositiveFinite(Acc_omega) && PositiveFinite(Dec_omega);
        if (!valid) {
            Param_valid_Flag = false;
            has_ref = false;
            generate_pending = false;
            output_b = {};
            Traj_complete_Flag = false;
            PointTrack_linear_complete_Flag = false;
            PointTrack_omega_complete_Flag = false;
            return;
        }

        const bool changed = !has_ref || !Param_valid_Flag ||
            fabsf(Ref.linear_x_ - ref.x) > 0.0015f ||
            fabsf(Ref.linear_y_ - ref.y) > 0.0015f ||
            fabsf(WrapYaw(Ref.omega_ - ref.yaw)) > 0.001f ||
            speed_In.Acc_linear != Traject_speed.Acc_linear ||
            speed_In.Dec_linear != Traject_speed.Dec_linear ||
            speed_In.v_Max != Traject_speed.v_Max ||
            speed_In.w_Max != Traject_speed.w_Max ||
            Acc_omega != acc_omega || Dec_omega != dec_omega;
        if (!changed) return;

        ref = {Ref.linear_x_, Ref.linear_y_, WrapYaw(Ref.omega_)};
        Traject_speed = speed_In;
        acc_omega = Acc_omega;
        dec_omega = Dec_omega;
        Param_valid_Flag = true;
        has_ref = true;
        generate_pending = true;
        Traj_complete_Flag = false;
        PointTrack_linear_complete_Flag = false;
        PointTrack_omega_complete_Flag = false;
    }

private:
    static constexpr uint16_t kPlanSegments = 128;
    static constexpr uint16_t kConstraintSubsamples = 4; // 每区间含两端点共 5 点
    static constexpr float_t kPlanDu = 1.0f / kPlanSegments;
    struct ProgressCell {
        float_t speed;
        float_t acc;
        float_t dec;
        float_t duration;
    };

    // =====================================================
    //  轨迹生成函数：建立曲线采样及随 u 变化的速度表
    // =====================================================
    void TrajGenerate() {
        start = now;
        start.yaw = WrapYaw(start.yaw);
        u_ref = 0.0f;
        u_dot = 0.0f;
        segment_index = 0;
        segment_time = 0.0f;
        linear_settle_time = 0.0f;
        omega_settle_time = 0.0f;
        Traj_wff = {};
        track_w = {};
        PID_Reset(&track_path_xy);
        PID_Reset(&track_lateral_xy);
        PID_Reset(&track_omega);
        // PID_Reset 清零了各 PID 的时戳；重新建立计时基准。
        (void)DWT_GetDeltaT(&track_path_xy.DWT_CNT);
        (void)DWT_GetDeltaT(&track_lateral_xy.DWT_CNT);
        (void)DWT_GetDeltaT(&track_omega.DWT_CNT);

        tx = ny = 1.0f;
        ty = nx = 0.0f;
        if (!PositiveFinite(trajectory.max_u_speed) ||
            !PositiveFinite(trajectory.max_u_acc) || !PositiveFinite(trajectory.max_u_dec) ||
            (!trajectory.pose && trajectory.derivatives)) {
            Invalidate();
            return;
        }

        TrajGeometry first{}, last{};
        if (!SampleGeometry(0.0f, first) || !SampleGeometry(1.0f, last) ||
            !SamePose(first.pose, start) || !SamePose(last.pose, ref)) {
            Invalidate();
            return;
        }
        // 仅默认直线可根据端点判空；自定义轨迹可以闭合或转出去再转回来。
        if (!trajectory.pose && ref.x == start.x && ref.y == start.y &&
            WrapYaw(ref.yaw - start.yaw) == 0.0f) {
            u_ref = 1.0f;
            Traj_complete_Flag = true;
            return;
        }
        // 本次同步完成预采样；只在切换目标/轨迹或 Restart 时执行，非每周期执行。
        if (!Build_ProgressPlan()) Invalidate();
    }

    // =====================================================
    //  曲线约束：q_dot=q' u_dot；q_ddot=q'' u_dot^2 + q' u_ddot
    //  对每个小区间取采样导数范数的上界，保守地分配曲率/进度加速度预算。
    //  这是离散近似，5% 余量不等于任意曲线的连续约束证明。
    //  窄峰/高频曲线需增加采样密度并验证；建议提供解析导数。
    // =====================================================
    static void Apply_DerivativeLimits(float_t d1, float_t d2,
        float_t v_max, float_t a_max, float_t d_max, ProgressCell& cell) {
        const float_t margin = 0.95f;
        const bool curved = d2 > 1e-6f;
        const float_t tangent_share = curved ? 0.5f : 1.0f;
        if (d1 > 1e-6f) {
            cell.speed = fminf(cell.speed, margin * v_max / d1);
            cell.acc = fminf(cell.acc, margin * tangent_share * a_max / d1);
            cell.dec = fminf(cell.dec, margin * tangent_share * d_max / d1);
        }
        if (curved) {
            const float_t curve_budget = margin * 0.5f * fminf(a_max, d_max);
            cell.speed = fminf(cell.speed, sqrtf(curve_budget / d2));
        }
    }

    bool Build_ProgressPlan() {
        for (uint16_t i = 0; i <= kPlanSegments; ++i)
            node_speed[i] = trajectory.max_u_speed;

        for (uint16_t i = 0; i < kPlanSegments; ++i) {
            ProgressCell& cell = plan[i];
            cell = {trajectory.max_u_speed, trajectory.max_u_acc, trajectory.max_u_dec, 0.0f};
            float_t linear_d1 = 0.0f, linear_d2 = 0.0f;
            float_t angular_d1 = 0.0f, angular_d2 = 0.0f;
            for (uint16_t j = 0; j <= kConstraintSubsamples; ++j) {
                const float_t u = (i + static_cast<float_t>(j) / kConstraintSubsamples) * kPlanDu;
                TrajGeometry sample{};
                if (!SampleGeometry(u, sample)) return false;
                const float_t d1 = NormXY(sample.dpose_du);
                const float_t d2 = NormXY(sample.d2pose_du2);
                if (!std::isfinite(d1) || !std::isfinite(d2)) return false;
                linear_d1 = fmaxf(linear_d1, d1);
                linear_d2 = fmaxf(linear_d2, d2);
                angular_d1 = fmaxf(angular_d1, fabsf(sample.dpose_du.yaw) * kDegToRad);
                angular_d2 = fmaxf(angular_d2, fabsf(sample.d2pose_du2.yaw) * kDegToRad);
                if (trajectory.speed_limit) {
                    const float_t local_limit = trajectory.speed_limit(u, trajectory.context);
                    if (!PositiveFinite(local_limit)) return false;
                    cell.speed = fminf(cell.speed, local_limit);
                }
            }
            Apply_DerivativeLimits(linear_d1, linear_d2,
                fminf(Traject_speed.v_Max, MAX_VELOCITY_LINEAR),
                Traject_speed.Acc_linear, Traject_speed.Dec_linear, cell);
            Apply_DerivativeLimits(angular_d1, angular_d2,
                fminf(Traject_speed.w_Max, MAX_VELOCITY_ANGULAR), acc_omega, dec_omega, cell);
            if (!PositiveFinite(cell.speed) || !PositiveFinite(cell.acc) || !PositiveFinite(cell.dec))
                return false;
            // 边界速度同时受相邻两区间限制。
            node_speed[i] = fminf(node_speed[i], cell.speed);
            node_speed[i + 1] = fminf(node_speed[i + 1], cell.speed);
        }

        node_speed[0] = node_speed[kPlanSegments] = 0.0f;
        // 正向扫描保证能从上一个节点加速到当前节点。
        for (uint16_t i = 0; i < kPlanSegments; ++i)
            node_speed[i + 1] = fminf(node_speed[i + 1],
                sqrtf(node_speed[i] * node_speed[i] + 2.0f * plan[i].acc * kPlanDu));
        // 反向扫描保证能在后续低速区/终点之前完成减速。
        for (int i = kPlanSegments - 1; i >= 0; --i)
            node_speed[i] = fminf(node_speed[i],
                sqrtf(node_speed[i + 1] * node_speed[i + 1] + 2.0f * plan[i].dec * kPlanDu));
        for (uint16_t i = 0; i < kPlanSegments; ++i) {
            plan[i].duration = 2.0f * kPlanDu / (node_speed[i] + node_speed[i + 1]);
            if (!PositiveFinite(plan[i].duration)) return false;
        }
        return true;
    }

    // =====================================================
    //  更新 u_ref：在速度表的区间内积分，不依赖实测 x/y 的变化量
    //  区间内进度加速度恒定，整段加速度随曲线约束变化；暂不限制 jerk。
    // =====================================================
    float_t Update_u_ref() {
        if (!Traj_complete_Flag) {
            segment_time += dt;
            while (segment_index < kPlanSegments && segment_time >= plan[segment_index].duration) {
                segment_time -= plan[segment_index].duration;
                ++segment_index;
            }
            if (segment_index >= kPlanSegments) {
                u_ref = 1.0f;
                u_dot = 0.0f;
                Traj_complete_Flag = true;
            } else {
                const float_t v0 = node_speed[segment_index];
                const float_t acc = (node_speed[segment_index + 1] - v0) / plan[segment_index].duration;
                u_dot = fmaxf(0.0f, v0 + acc * segment_time);
                u_ref = segment_index * kPlanDu + v0 * segment_time + 0.5f * acc * segment_time * segment_time;
            }
            u_ref = fminf(1.0f, fmaxf(0.0f, u_ref));
        }
        return u_ref;
    }

    // =====================================================
    //  轨迹速度更新函数：q_dot = dq/du * u_dot
    // =====================================================
    void Update_TarjSpeed() {
        Traj_wff.vx = dx_du * u_dot;
        Traj_wff.vy = dy_du * u_dot;
        Traj_wff.w  = dyaw_du * kDegToRad * u_dot;
    }

    // =====================================================
    //  点跟踪平移速度更新函数：沿用原来的切向/法向 PID
    // =====================================================
    void Update_TrackSpeed_linear(chassis_position Ref_tmp) {
        const float_t e_x = Ref_tmp.x - now.x;
        const float_t e_y = Ref_tmp.y - now.y;
        const float_t e_xy = sqrtf(e_x * e_x + e_y * e_y);
        const float_t e_path = tx * e_x + ty * e_y;
        const float_t e_lateral = nx * e_x + ny * e_y;

        linear_settle_time = (Traj_complete_Flag && e_xy < 0.03f)
            ? linear_settle_time + dt : 0.0f;
        PointTrack_linear_complete_Flag = linear_settle_time >= 0.01f;

        const float_t path_output = PID_Calculate(&track_path_xy, 0.0f, e_path);
        const float_t lateral_output = PID_Calculate(&track_lateral_xy, 0.0f, e_lateral);
        track_w.vx = path_output * tx + lateral_output * nx;
        track_w.vy = path_output * ty + lateral_output * ny;
    }

    // =====================================================
    //  点跟踪角速度更新函数
    // =====================================================
    void Update_TrackSpeed_omega(chassis_position Ref_tmp) {
        const float_t e_yaw = WrapYaw(Ref_tmp.yaw - now.yaw);
        omega_settle_time = (Traj_complete_Flag && fabsf(e_yaw) < 0.5f)
            ? omega_settle_time + dt : 0.0f;
        PointTrack_omega_complete_Flag = omega_settle_time >= 0.01f;
        track_w.w = kDegToRad * PID_Calculate(&track_omega, 0.0f, e_yaw);
    }

    // =====================================================
    //  位姿曲线：默认例子仍简单；自定义曲线由回调提供
    //  默认三函数与其解析导数成对维护；扩展轨迹请用 Set_Trajectory。
    // =====================================================
    float_t Get_x_u(float_t u) const { return start.x + u * (ref.x - start.x); }
    float_t Get_y_u(float_t u) const { return start.y + u * (ref.y - start.y); }
    float_t Get_yaw_u(float_t u) const { return start.yaw + u * WrapYaw(ref.yaw - start.yaw); }

    chassis_position SamplePose(float_t u) const {
        if (trajectory.pose) return trajectory.pose(u, start, ref, trajectory.context);
        return {Get_x_u(u), Get_y_u(u), Get_yaw_u(u)};
    }

    bool SampleGeometry(float_t u, TrajGeometry& sample) const {
        sample = {};
        sample.pose = SamplePose(u);
        if (!FinitePose(sample.pose)) return false;
        if (!trajectory.pose) {
            sample.dpose_du = {ref.x - start.x, ref.y - start.y, WrapYaw(ref.yaw - start.yaw)};
        } else if (trajectory.derivatives) {
            if (!trajectory.derivatives(u, start, ref, trajectory.context,
                    sample.dpose_du, sample.d2pose_du2)) return false;
        } else if (!NumericalDerivatives(u, sample)) {
            return false;
        }
        return FinitePose(sample.dpose_du) && FinitePose(sample.d2pose_du2);
    }

    // 对轨迹函数差分，而非对实测位置差分。端点用单边差分，不访问 [0,1] 外。
    // h 越小，float 舍入噪声对二阶导数越明显；精细轨迹推荐提供解析导数。
    // yaw 的相邻采样角度变化必须小于 180 deg，避免跨界差分歧义。
    bool NumericalDerivatives(float_t u, TrajGeometry& sample) const {
        constexpr float_t h = 1.0f / 256.0f;
        if (u >= h && u <= 1.0f - h) {
            const chassis_position p = SamplePose(u + h);
            const chassis_position m = SamplePose(u - h);
            if (!FinitePose(p) || !FinitePose(m)) return false;
            const chassis_position dp = PoseDifference(p, sample.pose);
            const chassis_position dm = PoseDifference(m, sample.pose);
            sample.dpose_du = {(dp.x - dm.x) / (2.0f * h), (dp.y - dm.y) / (2.0f * h),
                               (dp.yaw - dm.yaw) / (2.0f * h)};
            sample.d2pose_du2 = {(dp.x + dm.x) / (h * h), (dp.y + dm.y) / (h * h),
                                (dp.yaw + dm.yaw) / (h * h)};
        } else {
            const float_t direction = (u < h) ? 1.0f : -1.0f;
            const chassis_position p1 = SamplePose(u + direction * h);
            const chassis_position p2 = SamplePose(u + direction * 2.0f * h);
            const chassis_position p3 = SamplePose(u + direction * 3.0f * h);
            if (!FinitePose(p1) || !FinitePose(p2) || !FinitePose(p3)) return false;
            const chassis_position d1 = PoseDifference(p1, sample.pose);
            const chassis_position d2 = PoseDifference(p2, sample.pose);
            const chassis_position d3 = PoseDifference(p3, sample.pose);
            const float_t scale = direction / (2.0f * h);
            sample.dpose_du = {(4.0f * d1.x - d2.x) * scale, (4.0f * d1.y - d2.y) * scale,
                               (4.0f * d1.yaw - d2.yaw) * scale};
            sample.d2pose_du2 = {(4.0f * d2.x - 5.0f * d1.x - d3.x) / (h * h),
                                (4.0f * d2.y - 5.0f * d1.y - d3.y) / (h * h),
                                (4.0f * d2.yaw - 5.0f * d1.yaw - d3.yaw) / (h * h)};
        }
        return true;
    }

    bool Update_TrajGeometry(float_t u) {
        TrajGeometry sample{};
        if (!SampleGeometry(u, sample)) {
            Invalidate();
            return false;
        }
        Traj_u_ref = sample.pose;
        dx_du = sample.dpose_du.x;
        dy_du = sample.dpose_du.y;
        dyaw_du = sample.dpose_du.yaw;
        const float_t tangent_length = NormXY(sample.dpose_du);
        if (!std::isfinite(tangent_length)) {
            Invalidate();
            return false;
        }
        // 切法向随曲线更新；纯旋转/平移驻点沿用上一个有效基底。
        if (tangent_length > 1e-6f) {
            tx = dx_du / tangent_length;
            ty = dy_du / tangent_length;
            nx = -ty;
            ny = tx;
        }
        return true;
    }

    static chassis_position PoseDifference(chassis_position a, chassis_position b) {
        return {a.x - b.x, a.y - b.y, WrapYaw(a.yaw - b.yaw)};
    }

    static bool FinitePose(chassis_position value) {
        return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.yaw);
    }

    static float_t NormXY(chassis_position value) {
        return sqrtf(value.x * value.x + value.y * value.y);
    }

    static bool SamePose(chassis_position a, chassis_position b) {
        const chassis_position delta = PoseDifference(a, b);
        return fabsf(delta.x) <= 0.0015f && fabsf(delta.y) <= 0.0015f && fabsf(delta.yaw) <= 0.001f;
    }

    void Invalidate() {
        Param_valid_Flag = false;
        Traj_complete_Flag = false;
        PointTrack_linear_complete_Flag = false;
        PointTrack_omega_complete_Flag = false;
        u_dot = 0.0f;
        Traj_wff = {};
        track_w = {};
        output_w = {};
        output_b = {};
    }

    void Limit_Output() {
        // 规划的加速度约束作用于参考/前馈；叠加 PID 后这里只限制速度。
        const float_t v_limit = fminf(Traject_speed.v_Max, MAX_VELOCITY_LINEAR);
        const float_t v = sqrtf(output_w.vx * output_w.vx + output_w.vy * output_w.vy);
        if (v > v_limit) {
            output_w.vx *= v_limit / v;
            output_w.vy *= v_limit / v;
        }
        const float_t w_limit = fminf(Traject_speed.w_Max, MAX_VELOCITY_ANGULAR);
        output_w.w = fminf(w_limit, fmaxf(-w_limit, output_w.w));
    }

    static bool PositiveFinite(float_t value) {
        return std::isfinite(value) && value > 0.0f;
    }

    static float_t WrapYaw(float_t yaw) {
        yaw = fmodf(yaw, 360.0f);
        if (yaw > 180.0f) yaw -= 360.0f;
        if (yaw < -180.0f) yaw += 360.0f;
        return yaw;
    }

    float_t u_ref = 0.0f; // 无量纲，0~1
    float_t u_dot = 0.0f; // 1/s，与实际线速度无直接绑定关系
    TrajectorySource trajectory{};
    ProgressCell plan[kPlanSegments]{};
    float_t node_speed[kPlanSegments + 1]{};
    uint16_t segment_index = 0;
    float_t segment_time = 0.0f;

    chassis_position Traj_u_ref{};
    chassis_position ref{};
    chassis_position start{};
    chassis_position now{};

    float_t tx = 1.0f;
    float_t ty = 0.0f;
    float_t nx = 0.0f;
    float_t ny = 1.0f;
    float_t dx_du = 0.0f;
    float_t dy_du = 0.0f;
    float_t dyaw_du = 0.0f;

    speed_data Traject_speed{1.2f, 0.9f, 2.1f, 1.8849556f};
    float_t acc_omega = 3.14f;
    float_t dec_omega = 3.14f;
    chassis_speed Traj_wff{};
    chassis_speed track_w{};
    chassis_speed output_w{};
    chassis_speed output_b{};

    float_t dt = 0.0f;
    uint32_t DWT_CNT = 0;
    bool clock_ready = false;
    bool has_ref = false;
    bool generate_pending = false;
    float_t linear_settle_time = 0.0f;
    float_t omega_settle_time = 0.0f;

    PID_t track_path_xy{.Kp = 4.88f, .Ki = 0.01f, .Kd = 0.55f, .MaxOut = 0.95f * MAX_VELOCITY_LINEAR, .DeadBand = 0.005f, .Improve = NONE};
    PID_t track_lateral_xy{.Kp = 3.0f, .Ki = 0.03f, .Kd = 0.35f, .MaxOut = 0.95f * MAX_VELOCITY_LINEAR, .DeadBand = 0.005f, .Improve = NONE};
    PID_t track_omega{.Kp = 5.30f, .Ki = 0.3f, .Kd = 0.55f, .MaxOut = MAX_VELOCITY_ANGULAR * 0.75f / kDegToRad, .IntegralLimit = 50000.0f, .DeadBand = 0.1f, .Improve = Integral_Limit};
};
