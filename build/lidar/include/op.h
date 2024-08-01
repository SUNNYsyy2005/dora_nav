namespace tf{
    void getRPY(const Quaternion& m, double& roll, double& pitch, double& yaw) {
        const double epsilon = 1e-6;
        const double pi = 3.14159265358979323846;

        double sqx = m.x * m.x;
        double sqy = m.y * m.y;
        double sqz = m.z * m.z;

        // roll (x-axis rotation)
        double t0 = +2.0 * (m.w * m.x + m.y * m.z);
        double t1 = +1.0 - 2.0 * (sqx + sqy);
        roll = std::atan2(t0, t1);

        // pitch (y-axis rotation)
        double t2 = +2.0 * (m.w * m.y - m.z * m.x);
        t2 = t2 > 1.0 ? 1.0 : t2;
        t2 = t2 < -1.0 ? -1.0 : t2;
        pitch = std::asin(t2);

        // yaw (z-axis rotation)
        double t3 = +2.0 * (m.w * m.z + m.x * m.y);
        double t4 = +1.0 - 2.0 * (sqy + sqz);
        yaw = std::atan2(t3, t4);
    }
}