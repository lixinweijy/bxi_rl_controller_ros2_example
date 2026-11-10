// Read-only math wrapper for the exact deployment's parallel_joint.cpp.
void elf3_func(double, double, double, double, double, double, double*);
void elf3_j_func(double, double, double, double, double, double, double*);

extern "C" void xmz_parallel(const double* q, double* motor, double* jacobian) {
    elf3_func(q[0], q[1], q[2], q[3], q[4], q[5], motor);
    elf3_j_func(q[0], q[1], q[2], q[3], q[4], q[5], jacobian);
}
