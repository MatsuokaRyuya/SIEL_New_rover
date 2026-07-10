import math
from functools import partial

import rclpy
from rclpy.parameter import Parameter#osr_param.yaml
from rclpy.node import Node
import tf2_ros

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, TwistWithCovariance, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from osr_interfaces.msg import CommandDrive, CommandCorner# 速度をまとめて入れとく型のメッセージなんてないから作る


class Rover(Node):
    """Math and motor control algorithms to move the rover"""

    def __init__(self):
        super().__init__("rover")
        self.log = self.get_logger()
        # self.log.set_level(10)
        self.log.info("Initializing Rover")

        self.declare_parameters(
            namespace='',
            parameters=[
                ('rover_dimensions.d1', Parameter.Type.DOUBLE),
                ('rover_dimensions.d2', Parameter.Type.DOUBLE),
                ('rover_dimensions.d3', Parameter.Type.DOUBLE),
                ('rover_dimensions.d4', Parameter.Type.DOUBLE),
                ('rover_dimensions.wheel_radius', Parameter.Type.DOUBLE),
                ('drive_no_load_rpm', Parameter.Type.DOUBLE),
                ('enable_odometry', Parameter.Type.BOOL),
                ('publish_transform', Parameter.Type.BOOL)
            ]
        )
        self.d1 = self.get_parameter('rover_dimensions.d1').get_parameter_value().double_value
        self.d2 = self.get_parameter('rover_dimensions.d2').get_parameter_value().double_value
        self.d3 = self.get_parameter('rover_dimensions.d3').get_parameter_value().double_value
        self.d4 = self.get_parameter('rover_dimensions.d4').get_parameter_value().double_value

        self.min_radius = 0.45  # [m]
        self.max_radius = 6.4  # [m]

        self.no_cmd_thresh = 0.05  # [rad]
        self.wheel_radius = self.get_parameter(
            "rover_dimensions.wheel_radius").get_parameter_value().double_value  # [m]
        drive_no_load_rpm = self.get_parameter(
            "drive_no_load_rpm").get_parameter_value().double_value
        self.max_vel = self.wheel_radius * drive_no_load_rpm / 60 * 2 * math.pi  # [m/s]
        self.should_calculate_odom = self.get_parameter("enable_odometry").get_parameter_value().bool_value
        self.should_publish_transform = self.get_parameter("publish_transform").get_parameter_value().bool_value
        if self.should_calculate_odom:
            self.get_logger().info("Calculting wheel odometry and publishing to /odom topic")
            self.odometry = Odometry()
            self.odometry.header.stamp = self.get_clock().now().to_msg()
            self.odometry.header.frame_id = "odom"
            self.odometry.child_frame_id = "base_link"
            self.odometry.pose.pose.orientation.w = 1.
        self.curr_positions = {}
        self.curr_velocities = {}
        self.curr_twist = TwistWithCovariance()
        self.curr_turning_radius = self.max_radius

        self.cmd_vel_sub = self.create_subscription(Twist, "/cmd_vel", 
                                                    partial(self.cmd_cb, intuitive=False), 1)
        self.cmd_vel_int_sub = self.create_subscription(Twist, "/cmd_vel_intuitive", 
                                                        partial(self.cmd_cb, intuitive=True), 1)
        self.drive_enc_sub = self.create_subscription(JointState, "/drive_state", self.enc_cb, 1)
        self.corner_enc_sub = self.create_subscription(JointState, "/corner_state", self.enc_cb, 1)

        self.turning_radius_pub = self.create_publisher(Float64, "/turning_radius", 1)
        if self.should_calculate_odom:
            self.odometry_pub = self.create_publisher(Odometry, "/odom", 2)
            self.tf_pub = tf2_ros.TransformBroadcaster(self)

        self.corner_cmd_pub = self.create_publisher(CommandCorner, "/cmd_corner", 1)
        self.drive_cmd_pub = self.create_publisher(CommandDrive, "/cmd_drive", 1)

    def cmd_cb(self, twist_msg, intuitive=False):
        """
        Respond to an incoming Twist command in one of two ways depending on the mode (intuitive)

        The Mathematically correct mode (intuitive=False) means that 
         * when the linear velocity is zero, an angular velocity does not cause the corner motors to move
           (since simply steering the corners while standing still doesn't generate a twist)
         * when driving backwards, steering behaves opposite as what you intuitively might expect
           (this is to hold true to the commanded twist)
        Use this topic with a controller that generated velocities based on targets. When you're
        controlling the robot with a joystick or other manual input topic, consider using the 
        /cmd_vel_intuitive topic instead.

        The Intuitive mode (intuitive=True) means that sending a positive angular velocity (moving joystick left)
        will always make the corner wheels turn 'left' regardless of the linear velocity.

        :param intuitive: determines the mode
        """
        # check if we're supposed to rotate in place
        if twist_msg.angular.y and not twist_msg.linear.x:#0=false, 1=true コントローラからの指令で前進速度が0かつ旋回速度が0でない場合どっちもtrueだから、turn in place
            # command corners to point to center
            corner_cmd_msg, drive_cmd_msg = self.calculate_rotate_in_place_cmd(twist_msg)

        else:
            desired_turning_radius = self.twist_to_turning_radius(twist_msg, intuitive_mode=intuitive)
            self.get_logger().debug("desired turning radius: " + "{}".format(desired_turning_radius), throttle_duration_sec=1)
            corner_cmd_msg = self.calculate_corner_positions(desired_turning_radius)

            # if we're turning, calculate the max velocity the middle of the rover can go
            max_vel = abs(desired_turning_radius) / (abs(desired_turning_radius) + self.d1) * self.max_vel
            if math.isnan(max_vel):  # turning radius infinite, going straight （max_velが数じゃないときにtrue投げる）
                max_vel = self.max_vel
            velocity = min(max_vel, twist_msg.linear.x)
            self.get_logger().debug("velocity drive cmd: {} m/s".format(velocity), throttle_duration_sec=1)

            drive_cmd_msg = self.calculate_drive_velocities(velocity, desired_turning_radius)

        # if self.corner_cmd_threshold(corner_cmd_msg):
        self.get_logger().debug("drive cmd:\n{}".format(drive_cmd_msg), throttle_duration_sec=1)
        self.get_logger().debug("corner cmd:\n{}".format(corner_cmd_msg), throttle_duration_sec=1)

        self.corner_cmd_pub.publish(corner_cmd_msg)
        self.drive_cmd_pub.publish(drive_cmd_msg)

    def enc_cb(self, msg):
        """When we get a JointState message from the drive or corner motors"""#JointStateは車輪名前の文字列リストとpositionというサーボの角度の数値リスト、velocityという速度の値リストがばらばらのリストとして入ってる型、roboclaw_wrapperとservo_controlが中身をうまく入れてくれてる
        # merge dictionaries since we could get corner or drive motor feedback
        self.curr_positions = {**self.curr_positions, **dict(zip(msg.name, msg.position))}#msg.nameは車輪の場所に対応、msg.positionは車輪のステアリング角度、zip(,)は二つのリストの中身をその順番にペアにしていく関数で、dict()はキーと値のセット（辞書型）に変換する。
        self.curr_velocities = {**self.curr_velocities, **dict(zip(msg.name, msg.velocity))}#**は辞書型の展開に使う。辞書データの前につけることでキーと値をセットとした変数をアンパックできる、{,}で同じ名前のキーがあったら左を右で更新する
        if self.should_calculate_odom and len(self.curr_positions) == 8:# **10->8へ変更**odometry計算機能がONかつカレントポジション辞書の長さが10、velocityはサーボの方のmsgにはないからposition（サーボなら角度、ドライブなら回転した量）を長さ10で判定
            # measure how much time has elapsed since our last update
            now = self.get_clock().now()
            dt = float(now.nanoseconds - (self.odometry.header.stamp.sec*10**9 + self.odometry.header.stamp.nanosec)) / 10**9#経過時間（ナノ秒単位）
            self.forward_kinematics()#10個のモーターからのデータをもとにローバー全体の直進速度と旋回速度を計算し、self.curr_twistに保存
            dx = self.curr_twist.twist.linear.x * dt
            dth = self.curr_twist.twist.angular.z * dt
            # angle is straightforward: in 2D it's additive
            # first calculate the current_angle in the fixed frame
            current_angle = 2 * math.atan2(self.odometry.pose.pose.orientation.z, 
                                           self.odometry.pose.pose.orientation.w)
            new_angle = current_angle + dth
            self.odometry.pose.pose.orientation.z = math.sin(new_angle/2.)
            self.odometry.pose.pose.orientation.w = math.cos(new_angle/2.)
            # the new pose in x and y depends on the current heading
            self.odometry.pose.pose.position.x += math.cos(new_angle) * dx
            self.odometry.pose.pose.position.y += math.sin(new_angle) * dx
            self.odometry.pose.covariance = 36 * [0.0,]
            # explanation for values at https://www.freedomrobotics.ai/blog/tuning-odometry-for-wheeled-robots
            self.odometry.twist.covariance[0] = 0.0225
            self.odometry.twist.covariance[5] = 0.01
            self.odometry.twist.covariance[-5] = 0.0225
            self.odometry.twist.covariance[-1] = 0.04
            self.odometry.twist = self.curr_twist
            self.odometry.header.stamp = now.to_msg()
            self.odometry_pub.publish(self.odometry)
            if self.should_publish_transform: 
                transform_msg = TransformStamped()
                transform_msg.header.frame_id = "odom"
                transform_msg.child_frame_id = "base_link"
                transform_msg.header.stamp = now.to_msg()
                transform_msg.transform.translation.x = self.odometry.pose.pose.position.x
                transform_msg.transform.translation.y = self.odometry.pose.pose.position.y
                transform_msg.transform.rotation = self.odometry.pose.pose.orientation
                self.tf_pub.sendTransform(transform_msg)


    def corner_cmd_threshold(self, corner_cmd):
        try:
            if abs(corner_cmd.left_front_pos - self.curr_positions["corner_left_front"]) > self.no_cmd_thresh:
                return True
            elif abs(corner_cmd.left_back_pos - self.curr_positions["corner_left_back"]) > self.no_cmd_thresh:
                return True
            elif abs(corner_cmd.right_back_pos - self.curr_positions["corner_right_back"]) > self.no_cmd_thresh:
                return True
            elif abs(corner_cmd.right_front_pos - self.curr_positions["corner_right_front"]) > self.no_cmd_thresh:
                return True
            else:
                return False
        except AttributeError:  # haven't received current encoder positions yet
            return True

    def calculate_drive_velocities(self, speed, current_radius):
        """
        Calculate target velocities for the drive motors based on desired speed and current turning radius

        :param speed: Drive speed command range from -max_vel to max_vel, with max vel depending on the turning radius
        :param radius: Current turning radius in m
        """
        # clip the value to the maximum allowed velocity
        speed = max(-self.max_vel, min(self.max_vel, speed))#
        cmd_msg = CommandDrive()
        if speed == 0:
            return cmd_msg

        elif abs(current_radius) >= self.max_radius:  # Very large turning radius, all wheels same speed、まっすぐ走ってるときかな
            angular_vel = speed / self.wheel_radius
            cmd_msg.left_front_vel = angular_vel
            cmd_msg.left_middle_vel = angular_vel
            cmd_msg.left_back_vel = angular_vel
            cmd_msg.right_back_vel = -angular_vel#全部前進する方向にするために、モーターの向き的にマイナスになる
            cmd_msg.right_middle_vel = -angular_vel
            cmd_msg.right_front_vel = -angular_vel

            return cmd_msg

        else:
            # for the calculations, we assume positive radius (turn left) and adjust later ローバー全体の旋回半径から、それぞれの車輪が駆動する角速度に落とし込んでる
            radius = abs(current_radius)
            # the entire vehicle moves with the same angular velocity dictated by the desired speed,
            # around the radius of the turn. v = r * omega
            angular_velocity_center = float(speed) / radius
            # calculate desired velocities of all centers of wheels. Corner wheels on the same side
            # move with the same velocity. v = r * omega again
            vel_middle_closest = (radius - self.d4) * angular_velocity_center
            vel_corner_closest = math.hypot(radius - self.d1, self.d3) * angular_velocity_center
            vel_corner_farthest = math.hypot(radius + self.d1, self.d3) * angular_velocity_center
            vel_middle_farthest = (radius + self.d4) * angular_velocity_center

            # now from these desired velocities, calculate the desired angular velocity of each wheel
            # v = r * omega again
            ang_vel_middle_closest = vel_middle_closest / self.wheel_radius
            ang_vel_corner_closest = vel_corner_closest / self.wheel_radius
            ang_vel_corner_farthest = vel_corner_farthest / self.wheel_radius
            ang_vel_middle_farthest = vel_middle_farthest / self.wheel_radius

            if current_radius > 0:  # turning left
                cmd_msg.left_front_vel = ang_vel_corner_closest
                cmd_msg.left_back_vel = ang_vel_corner_closest
                cmd_msg.left_middle_vel = ang_vel_middle_closest
                cmd_msg.right_back_vel = -ang_vel_corner_farthest
                cmd_msg.right_front_vel = -ang_vel_corner_farthest
                cmd_msg.right_middle_vel = -ang_vel_middle_farthest
            else:  # turning right
                cmd_msg.left_front_vel = ang_vel_corner_farthest
                cmd_msg.left_back_vel = ang_vel_corner_farthest
                cmd_msg.left_middle_vel = ang_vel_middle_farthest
                cmd_msg.right_back_vel = -ang_vel_corner_closest
                cmd_msg.right_front_vel = -ang_vel_corner_closest
                cmd_msg.right_middle_vel = -ang_vel_middle_closest

            return cmd_msg

    def calculate_corner_positions(self, radius):#旋回半径に必要になる各サーボモーターの回転量を計算する
        """
        Takes a turning radius and computes the required angle for each corner motor

        A small turning radius means a sharp turn
        A large turning radius means mostly straight. Any radius larger than max_radius is essentially straight
        because of the encoders' resolution

        The positions are expressed in the motor's frame with the positive z-axis pointing down. This means
        that a positive angle corresponds to a right turn

        :param radius: positive value means turn left. 0.45 < abs(turning_radius) < inf
        """
        cmd_msg = CommandCorner()

        if radius >= self.max_radius:
            return cmd_msg  # assume straight

        theta_front_closest = math.atan2(self.d3, abs(radius) - self.d1)
        theta_front_farthest = math.atan2(self.d3, abs(radius) + self.d1)

        if radius > 0:
            cmd_msg.left_front_pos = -theta_front_closest
            cmd_msg.left_back_pos = theta_front_closest
            cmd_msg.right_back_pos = theta_front_farthest
            cmd_msg.right_front_pos = -theta_front_farthest
        else:
            cmd_msg.left_front_pos = theta_front_farthest
            cmd_msg.left_back_pos = -theta_front_farthest
            cmd_msg.right_back_pos = -theta_front_closest
            cmd_msg.right_front_pos = theta_front_closest

        return cmd_msg

    def calculate_rotate_in_place_cmd(self, twist):#turn in place
        """
        Calculate corner angles and drive motor speeds to rotate the robot in place (turning radius 0)
        """
        # TODO these are always the same, should use cache or calculate on parameter change
        corner_cmd = CommandCorner()
        corner_cmd.left_front_pos = math.atan(self.d3/self.d1)
        corner_cmd.left_back_pos = -corner_cmd.left_front_pos
        corner_cmd.right_back_pos = math.atan(self.d2/self.d1)
        corner_cmd.right_front_pos = -corner_cmd.right_back_pos

        drive_cmd = CommandDrive()
        angular_vel = twist.angular.y
        # velocity of each wheel center = angular velocity of center of rover * distance to wheel center
        front_wheel_vel = math.hypot(self.d1, self.d3) * angular_vel / self.wheel_radius
        drive_cmd.left_front_vel = front_wheel_vel
        drive_cmd.right_front_vel = front_wheel_vel
        back_wheel_vel = math.hypot(self.d1, self.d2) * angular_vel / self.wheel_radius
        drive_cmd.left_back_vel = back_wheel_vel
        drive_cmd.right_back_vel = back_wheel_vel
        middle_wheel_vel = self.d4 * angular_vel / self.wheel_radius
        drive_cmd.left_middle_vel = middle_wheel_vel
        drive_cmd.right_middle_vel = middle_wheel_vel

        return corner_cmd, drive_cmd 

    def twist_to_turning_radius(self, twist, clip=True, intuitive_mode=False):#旋回指令から実際の旋回半径を計算する
        """
        Convert a commanded twist into an actual turning radius

        ackermann steering: if l is distance travelled, rho the turning radius, and theta the heading of the middle of the robot,
        then: dl = rho * dtheta. With dt -> 0, dl/dt = rho * dtheta/dt
        dl/dt = twist.linear.x, dtheta/dt = twist.angular.z

        :param twist: geometry_msgs/Twist. Only linear.x and angular.z are used
        :param clip: whether the values should be clipped from min_radius to max_radius
        :param intuitive_mode: whether the turning radius should be mathematically correct (see cmd_cb()) or intuitive
        :return: physical turning radius in meter, clipped to the rover's limits
        """
        try:
            if intuitive_mode and twist.linear.x < 0:
                radius = twist.linear.x / -twist.angular.z
            else:
                radius = twist.linear.x / twist.angular.z
        except ZeroDivisionError:
                return float("Inf")

        # clip values so they lie in (-max_radius, -min_radius) or (min_radius, max_radius)
        if not clip:
            return radius
        if radius == 0:
            if intuitive_mode:
                if twist.angular.z == 0:
                    return self.max_radius
                else:
                    radius = self.min_radius * self.max_vel / twist.angular.z  # proxy
            else:  # mathematical mode: standing still, so can't generate an angular velocity
                return self.max_radius  
        if radius > 0:
            radius = max(self.min_radius, min(self.max_radius, radius))
        else:
            radius = max(-self.max_radius, min(-self.min_radius, radius))

        return radius

    def angle_to_turning_radius(self, angle):#ローバーの前方中央に仮想のwheelがあるとして、それの角度から旋回半径を求める
        """
        Convert the angle of a virtual wheel positioned in the middle of the front two wheels to a turning radius
        Turning left and positive angle corresponds to a positive turning radius

        :param angle: [-pi/4, pi/4]
        :return: turning radius for the given angle in [m]
        """
        try:
            radius = self.d3 / math.tan(angle)
        except ZeroDivisionError:
            return float("Inf")

        return radius

    def forward_kinematics(self):#エンコーダとかサーボの回転量から実際のローバーの動きを順運動学するメソッド、４輪版に変更する
        """
        Calculate current twist of the rover given current drive and corner motor velocities
        Also approximate current turning radius.

        Note that forward kinematics means solving an overconstrained system since the corner 
        motors may not be aligned perfectly and drive velocities might fight each other
        """
        # calculate current turning radius according to each corner wheel's angle
        # corner motor angles should be flipped since different coordinate axes in this node (positive z up)
        theta_fl = -self.curr_positions['corner_left_front']
        theta_fr = -self.curr_positions['corner_right_front']
        theta_bl = -self.curr_positions['corner_left_back']
        theta_br = -self.curr_positions['corner_right_back']
        # sum wheel angles to find out which direction the rover is mostly turning in
        if theta_fl + theta_fr + theta_bl + theta_br > 0:  # turning left
            r_front_closest = self.d1 + self.angle_to_turning_radius(theta_fl)
            r_front_farthest = -self.d1 + self.angle_to_turning_radius(theta_fr)
            r_back_closest = -self.d1 - self.angle_to_turning_radius(theta_bl)
            r_back_farthest = self.d1 - self.angle_to_turning_radius(theta_br)
        else:  # turning right
            r_front_farthest = self.d1 + self.angle_to_turning_radius(theta_fl)
            r_front_closest = -self.d1 + self.angle_to_turning_radius(theta_fr)
            r_back_farthest = -self.d1 - self.angle_to_turning_radius(theta_bl)
            r_back_closest = self.d1 - self.angle_to_turning_radius(theta_br)
        # get a best estimate of the turning radius by taking the median value (avg sensitive to outliers)
        approx_turning_radius = sum(sorted([r_front_farthest, r_front_closest, r_back_farthest, r_back_closest])[1:3])/2.0

        if math.isnan(approx_turning_radius):
            approx_turning_radius = self.max_radius
        self.get_logger().debug("Current approximate turning radius: {}".format(round(approx_turning_radius, 2)), throttle_duration_sec=1)
        self.curr_turning_radius = approx_turning_radius

        # we know that the linear velocity in x direction is the instantaneous velocity of the middle virtual
        # wheel which spins at the average speed of the two middle outer wheels.
        drive_angular_velocity = (self.curr_velocities['drive_left_middle'] + self.curr_velocities['drive_right_middle']) / 2.
        self.curr_twist.twist.linear.x = drive_angular_velocity * self.wheel_radius
        # now calculate angular velocity from its relation with linear velocity and turning radius
        try:
            self.curr_twist.twist.angular.z = self.curr_twist.twist.linear.x / self.curr_turning_radius
        except ZeroDivisionError:  # turn in place
            self.curr_twist.twist.linear.x = 0.0  # No linear motion
            # Angular velocity from wheel velocities. We subtract velocities because wheels are spinning in opposite directions
            # divide by two to average across the two middle wheels
            drive_angular_velocity = (self.curr_velocities['drive_left_middle'] - self.curr_velocities['drive_right_middle']) / 2.0
            self.curr_twist.twist.angular.z = drive_angular_velocity * self.wheel_radius / self.d4  # Use width from middle wheel to center of rover
            self.get_logger().debug(f"Turn-in-place detected. Angular velocity: {self.curr_twist.twist.angular.z}", throttle_duration_sec=1)

"""
def forward_kinematics(self):#４輪バージョン
        
        #Calculate current twist of the 4-wheel independent drive and steering rover.
        
        # 1. 4つのサーボの現在の実際の角度を取得（向きの反転を考慮）
        theta_fl = -self.curr_positions.get('corner_left_front', 0.0)
        theta_fr = -self.curr_positions.get('corner_right_front', 0.0)
        theta_bl = -self.curr_positions.get('corner_left_back', 0.0)
        theta_br = -self.curr_positions.get('corner_right_back', 0.0)

        # 2. 4つの駆動輪の現在の実際の回転速度（rad/s）を取得
        v_fl = self.curr_velocities.get('drive_left_front', 0.0) * self.wheel_radius
        v_fr = self.curr_velocities.get('drive_right_front', 0.0) * self.wheel_radius
        v_bl = self.curr_velocities.get('drive_left_back', 0.0) * self.wheel_radius
        v_br = self.curr_velocities.get('drive_right_back', 0.0) * self.wheel_radius

        # 3. 各車輪の「進行方向（X方向）」の実績速度を三角関数で割り出す
        # タイヤの速度ベクトルをロボットの正面方向（cos）に分解します
        v_x_fl = v_fl * math.cos(theta_fl)
        v_x_fr = v_fr * math.cos(theta_fr)
        v_x_bl = v_bl * math.cos(theta_bl)
        v_x_br = v_br * math.cos(theta_br)

        # 4. ローバー全体の現在の直進速度（linear.x）は、4輪の正面速度の平均値
        self.curr_twist.twist.linear.x = (v_x_fl + v_x_fr + v_x_bl + v_x_br) / 4.0

        # 5. ローバー全体の現在の旋回速度（angular.z）を計算する
        # トレッド幅の半分をd1（左右間隔の半分）、ホイールベースの半分をd3（前後間隔の半分）と仮定
        # 各タイヤの速度差と向きから、中心まわりの回転（角速度）を逆算します
        try:
            # 左右の速度差による回転成分を、機体の寸法パラメータ（self.d1, self.d3）で割る
            # 前輪側の回転推測
            omega_front = (v_fl * math.sin(theta_fl) - v_fr * math.sin(theta_fr)) / (2.0 * self.d1)
            # 後輪側の回転推測
            omega_back = (v_bl * math.sin(theta_bl) - v_br * math.sin(theta_br)) / (2.0 * self.d1)
            
            # 左右の駆動速度差による標準的な差動回転成分（直進成分からの回転）
            omega_drive = ((v_x_fr - v_x_fl) + (v_x_br - v_x_bl)) / (4.0 * self.d1)
            
            # これらを総合的に平均して、もっともらしい全体の旋回角速度（rad/s）を決定
            self.curr_twist.twist.angular.z = (omega_front + omega_back + omega_drive) / 3.0

        except ZeroDivisionError:
            self.curr_twist.twist.angular.z = 0.0
"""

def main(args=None):
    rclpy.init(args=args)

    rover = Rover()

    rclpy.spin(rover)
    rover.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
