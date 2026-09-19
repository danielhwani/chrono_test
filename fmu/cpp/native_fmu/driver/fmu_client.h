/*
 * fmu_client -- a small reusable library that does the FMI2 Co-Simulation
 * "master" plumbing (dlopen the model .so, parse modelDescription.xml for
 * the GUID/modelIdentifier/value-references, run the instantiate ->
 * setupExperiment -> enter/exitInitializationMode sequence, then expose
 * plain setReal/getReal/doStep/close calls) as a small C API instead of a
 * standalone program.
 *
 * This is fmu_driver.c's own FMI-loading logic, pulled out so it's not
 * stuck inside a `main()` that only a CLI tool can use. Two consumers:
 *   - fmu_driver.c (this directory): the CLI tool, now just argv parsing +
 *     bench/csv/paced loops on top of this library.
 *   - the planned ros2_control ChronoFmuSystemInterface plugin: a
 *     hardware_interface::SystemInterface that talks to a vehicle FMU from
 *     inside the controller_manager process. It needs the exact same
 *     open/find_vr/set_real/get_real/do_step/close calls fmu_driver.c
 *     already had inline -- rather than copy that code into the plugin
 *     (and then have two copies to keep in sync), both link this library.
 *
 * extern "C" throughout (see the __cplusplus guard below) so a C++ plugin
 * can include and link this without name-mangling issues -- it's still
 * built as plain C (fmu_client.c has no C++ in it), just callable from C++.
 *
 * Build: compile fmu_client.c alongside your program and link -ldl (dlopen).
 */
#ifndef FMU_CLIENT_H
#define FMU_CLIENT_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef unsigned int FmuValueReference;
typedef struct FmuClient FmuClient;

/* Opens an FMU from an already-extracted directory (modelDescription.xml +
 * binaries/linux64/<modelIdentifier>.so -- what you get by unzipping a
 * .fmu, or what native_fmu/build.sh-style scripts in this repo leave in
 * place alongside the .fmu they also produce). Runs fmi2Instantiate ->
 * fmi2SetupExperiment -> fmi2EnterInitializationMode -> fmi2ExitInitializationMode
 * before returning, so the FMU is ready for fmu_client_set_real()/
 * fmu_client_do_step() immediately. instance_name is passed through to
 * fmi2Instantiate (informational only, doesn't need to be unique).
 *
 * Returns NULL on any failure (a diagnostic is printed to stderr).
 */
FmuClient* fmu_client_open(const char* fmu_dir, const char* instance_name);

/* Looks up a Real variable's value reference by name, from the FMU's own
 * modelDescription.xml. Returns 1 and writes *out_vr on success, 0 if no
 * variable with that name exists. Safe to call any time after open(),
 * repeatedly, for as many variable names as needed. */
int fmu_client_find_vr(const FmuClient* client, const char* var_name, FmuValueReference* out_vr);

/* fmi2SetReal / fmi2GetReal / fmi2DoStep passthroughs. Return 1 on fmi2OK,
 * 0 otherwise (matching this library's own success/failure convention,
 * not the raw FMI2 status codes). noSetFMUStatePriorToCurrentPoint is
 * always passed as true (this library never rewinds/re-steps). */
int fmu_client_set_real(FmuClient* client, const FmuValueReference* vrs, size_t n, const double* values);
int fmu_client_get_real(FmuClient* client, const FmuValueReference* vrs, size_t n, double* values);
int fmu_client_do_step(FmuClient* client, double current_time, double step_size);

/* fmi2Terminate + fmi2FreeInstance + dlclose, then frees the FmuClient
 * itself. client may be NULL (no-op). */
void fmu_client_close(FmuClient* client);

/* Diagnostics -- the GUID/modelIdentifier this client parsed out of
 * modelDescription.xml, mainly for printing a "model: X guid: Y" banner
 * like fmu_driver.c does. Valid for the client's lifetime; do not free. */
const char* fmu_client_model_id(const FmuClient* client);
const char* fmu_client_guid(const FmuClient* client);

#ifdef __cplusplus
}
#endif

#endif /* FMU_CLIENT_H */
