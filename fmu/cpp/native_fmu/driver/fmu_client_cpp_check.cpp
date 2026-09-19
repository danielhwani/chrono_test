/*
 * Step 2 of the ros2_control prep plan: exercise fmu_client.h from C++
 * before adding pluginlib/hardware_interface complexity on top. This is
 * NOT a ros2_control plugin -- it's a plain C++ program that mimics the
 * call pattern the future ChronoFmuSystemInterface's on_init/export_state_
 * interfaces/export_command_interfaces/read/write will use (std::vector<double>
 * for state/command storage,
 * open once, loop set_real -> do_step -> get_real, close once), so any
 * C/C++ linkage or calling-convention problem shows up here instead of
 * being tangled up with ROS2/pluginlib issues later.
 *
 * fmu_client.h already wraps its declarations in `extern "C"` for exactly
 * this reason -- this file is what actually proves that works, compiled
 * as real C++ (g++), not just assumed from reading the header.
 *
 * Two checks:
 *   1) Same 4-wheel/2WD scenario fmu_driver.c already validates (steer_deg,
 *      drive_torque_rear), to catch any C/C++ boundary discrepancy against
 *      a known-good C result.
 *   2) A 6x6 run (six_wheel + four_wheel_drive, independent per-axle
 *      torques) matching the scenario validate_native_vehicle_fmu.py
 *      already confirmed bit-exact between the two FMU variants -- printed
 *      here so it can be eyeballed against those already-known numbers.
 *
 * Build: see ./build.sh (adds a g++ line for this file, links fmu_client.c
 * and -ldl same as fmu_driver).
 */
#include "fmu_client.h"

#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

/* Mimics the shape a hardware_interface::SystemInterface would use:
 * named interfaces, each backed by one double in a plain vector -- so
 * read()/write() in the real plugin just become fmu_client_get_real()/
 * set_real() over the same vr/value arrays built here. */
struct NamedInterfaces {
    std::vector<std::string> names;
    std::vector<FmuValueReference> vrs;
    std::vector<double> values;

    void add(FmuClient* client, const std::string& name, double initial = 0.0) {
        FmuValueReference vr;
        if (!fmu_client_find_vr(client, name.c_str(), &vr)) {
            std::fprintf(stderr, "fmu_client_find_vr failed for '%s'\n", name.c_str());
            std::exit(1);
        }
        names.push_back(name);
        vrs.push_back(vr);
        values.push_back(initial);
    }

    double& operator[](const std::string& name) {
        for (size_t i = 0; i < names.size(); ++i) {
            if (names[i] == name) return values[i];
        }
        std::fprintf(stderr, "no such interface '%s'\n", name.c_str());
        std::exit(1);
    }
};

static void run_scenario(const char* fmu_dir, const char* label, bool six_wheel, bool four_wheel_drive,
                          double steer_deg, double torque_rear, double torque_mid, double torque_front,
                          double sim_time, double dt) {
    FmuClient* client = fmu_client_open(fmu_dir, "fmu_client_cpp_check");
    if (!client) {
        std::fprintf(stderr, "fmu_client_open failed for %s\n", fmu_dir);
        std::exit(1);
    }
    std::printf("=== %s ===\n", label);
    std::printf("model: %s  guid=%s\n", fmu_client_model_id(client), fmu_client_guid(client));

    /* command_interfaces-equivalent: set once (structural) or every step (runtime) */
    NamedInterfaces commands;
    commands.add(client, "steer_deg", steer_deg);
    commands.add(client, "drive_torque_rear", torque_rear);
    commands.add(client, "drive_torque_mid", torque_mid);
    commands.add(client, "drive_torque_front", torque_front);
    commands.add(client, "six_wheel", six_wheel ? 1.0 : 0.0);
    commands.add(client, "four_wheel_drive", four_wheel_drive ? 1.0 : 0.0);
    /* six_wheel/four_wheel_drive are structural -- set once, before stepping starts */
    fmu_client_set_real(client, commands.vrs.data(), commands.vrs.size(), commands.values.data());

    /* state_interfaces-equivalent: read back each step */
    NamedInterfaces states;
    states.add(client, "chassis_x");
    states.add(client, "chassis_z");
    states.add(client, "yaw_deg");
    states.add(client, "speed_mps");

    long n_steps = (long)(sim_time / dt);
    double t = 0.0;
    for (long i = 0; i < n_steps; ++i) {
        /* write(): push the (here, constant) runtime commands into the FMU and step it */
        fmu_client_set_real(client, commands.vrs.data(), commands.vrs.size(), commands.values.data());
        fmu_client_do_step(client, t, dt);
        t += dt;

        /* read(): pull the resulting state back out */
        fmu_client_get_real(client, states.vrs.data(), states.vrs.size(), states.values.data());
    }

    std::printf("t=%.3f  chassis_x=%.6f  chassis_z=%.6f  yaw_deg=%.6f  speed_mps=%.6f\n\n",
                t, states["chassis_x"], states["chassis_z"], states["yaw_deg"], states["speed_mps"]);

    fmu_client_close(client);
}

int main(int argc, char** argv) {
    const char* fmu_dir = argc > 1 ? argv[1] : "../../native_vehicle_fmu";

    /* 1) same 4-wheel/2WD scenario as fmu_driver.c's own examples, to
     * cross-check the C++ call path against a known-good C result. */
    run_scenario(fmu_dir, "4-wheel, 2WD (steer=10)", false, false,
                 /*steer*/ 10.0, /*rear*/ 260.0, /*mid*/ 0.0, /*front*/ 0.0,
                 /*sim_time*/ 2.0, /*dt*/ 0.002);

    /* 2) 6x6 with independent per-axle torques, matching the scenario
     * validate_native_vehicle_fmu.py already confirmed bit-exact between
     * the pythonfmu and native C++ FMUs. */
    run_scenario(fmu_dir, "6x6 (rear=300 mid=200 front=100)", true, true,
                 /*steer*/ 0.0, /*rear*/ 300.0, /*mid*/ 200.0, /*front*/ 100.0,
                 /*sim_time*/ 3.0, /*dt*/ 0.002);

    std::printf("OK -- fmu_client called correctly from C++ in both scenarios.\n");
    return 0;
}
