void print(LaserScan scan) {
    printf("seq: %d\n", scan.header.seq);
    printf("stamp: %lld.%lld\n", scan.header.stamp.sec, scan.header.stamp.nsec);
    printf("frame_id: %s\n", scan.header.frame_id.c_str());
    printf("angle_min: %f\n", scan.angle_min);
    printf("angle_max: %f\n", scan.angle_max);
    printf("angle_increment: %f\n", scan.angle_increment);
    printf("time_increment: %f\n", scan.time_increment);
    printf("scan_time: %f\n", scan.scan_time);
    printf("range_min: %f\n", scan.range_min);
    printf("range_max: %f\n", scan.range_max);
    printf("ranges: ");
    for (float range : scan.ranges) {
        printf("%f ", range);
    }
    printf("\n");
    printf("intensities: ");
    for (float intensity : scan.intensities) {
        printf("%f ", intensity);
    }
    printf("\n");
}
void print(Imu imu){
        printf("seq: %d\n", imu.header.seq);
        printf("stamp: %lld.%lld\n", imu.header.stamp.sec, imu.header.stamp.nsec);
        printf("frame_id: %s\n", imu.header.frame_id.c_str());
        printf("orientation: x:%f y:%f z:%f w:%f\n", imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w);
        printf("orientation_covariance: ");
        for (double cov : imu.orientation_covariance) {
            printf("%f ", cov);
        }
        printf("\n");
        printf("angular_velocity: x:%f y:%f z:%f\n", imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z);
        printf("angular_velocity_covariance: ");
        for (double cov : imu.angular_velocity_covariance) {
            printf("%f ", cov);
        }
        printf("\n");
        printf("linear_acceleration: x:%f y:%f z:%f\n", imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z);
        printf("linear_acceleration_covariance: ");
        for (double cov : imu.linear_acceleration_covariance) {
            printf("%f ", cov);
        }
        printf("\n");
    }
void print(Pose2D pose){
        printf("x:%f y:%f z:%f\n",pose.x,pose.y,pose.theta);
    }
void print(TwistWithCovariance twist){
        printf("linear.x:%f  linear.y:%f  linear.z:%f\n",twist.twist.linear.x,twist.twist.linear.y,twist.twist.linear.z);
        printf("angular.x:%f  angular.y:%f  angular.z:%f\n",twist.twist.angular.x,twist.twist.angular.y,twist.twist.angular.z);
        for (double cov : twist.covariance) {
            printf("%f ", cov);
        }
        printf("\n");
    }