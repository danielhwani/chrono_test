/*
 * fmu_client.c -- implementation. See fmu_client.h for the API and why this
 * exists as a library instead of code inlined into fmu_driver.c's main().
 *
 * The FMI2 type definitions, the tiny modelDescription.xml scanner, and the
 * real (non-NULL) callback functions below are the same ones fmu_driver.c
 * used to have inline -- moved here verbatim, not rewritten, so behavior is
 * unchanged (see the git history of fmu_driver.c for how these were
 * developed/debugged, e.g. why the callbacks can't be NULL for
 * OpenModelica-generated FMUs).
 */
#include "fmu_client.h"

#include <dlfcn.h>
#include <limits.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ---- minimal FMI2 type definitions (no official fmi2Functions.h vendored
 * in this repo -- see fmu/cpp/native_fmu/sources/bouncing_ball_native.c's
 * header comment for why hand-writing these from the spec is fine) ---- */
typedef void* fmi2Component;
typedef void* fmi2ComponentEnvironment;
typedef const char* fmi2String;
typedef double fmi2Real;
typedef int fmi2Integer;
typedef int fmi2Boolean;
typedef unsigned int fmi2ValueReference;
typedef int fmi2Status;
typedef int fmi2Type;

#define fmi2OK 0
#define fmi2CoSimulation 1

typedef void (*fmi2CallbackLogger)(fmi2ComponentEnvironment, fmi2String, fmi2Status, fmi2String, fmi2String, ...);
typedef void* (*fmi2CallbackAllocateMemory)(size_t, size_t);
typedef void (*fmi2CallbackFreeMemory)(void*);
typedef void (*fmi2StepFinished)(fmi2ComponentEnvironment, fmi2Status);

typedef struct {
    fmi2CallbackLogger logger;
    fmi2CallbackAllocateMemory allocateMemory;
    fmi2CallbackFreeMemory freeMemory;
    fmi2StepFinished stepFinished;
    fmi2ComponentEnvironment componentEnvironment;
} fmi2CallbackFunctions;

typedef fmi2Component (*fmi2Instantiate_t)(fmi2String, fmi2Type, fmi2String, fmi2String,
                                            const fmi2CallbackFunctions*, fmi2Boolean, fmi2Boolean);
typedef void (*fmi2FreeInstance_t)(fmi2Component);
typedef fmi2Status (*fmi2SetupExperiment_t)(fmi2Component, fmi2Boolean, fmi2Real, fmi2Real, fmi2Boolean, fmi2Real);
typedef fmi2Status (*fmi2EnterInitializationMode_t)(fmi2Component);
typedef fmi2Status (*fmi2ExitInitializationMode_t)(fmi2Component);
typedef fmi2Status (*fmi2Terminate_t)(fmi2Component);
typedef fmi2Status (*fmi2SetReal_t)(fmi2Component, const fmi2ValueReference[], size_t, const fmi2Real[]);
typedef fmi2Status (*fmi2GetReal_t)(fmi2Component, const fmi2ValueReference[], size_t, fmi2Real[]);
typedef fmi2Status (*fmi2DoStep_t)(fmi2Component, fmi2Real, fmi2Real, fmi2Boolean);

struct FmuClient {
    void* handle;       /* dlopen handle */
    char* xml;           /* modelDescription.xml contents, kept for find_vr */
    char guid[128];
    char model_id[128];
    fmi2Component component;

    fmi2FreeInstance_t fmi2FreeInstance;
    fmi2SetReal_t fmi2SetReal;
    fmi2GetReal_t fmi2GetReal;
    fmi2DoStep_t fmi2DoStep;
    fmi2Terminate_t fmi2Terminate;
};

static void* xdlsym(void* handle, const char* name) {
    void* sym = dlsym(handle, name);
    if (!sym) {
        fprintf(stderr, "dlsym failed for %s: %s\n", name, dlerror());
        exit(1);
    }
    return sym;
}

static char* read_whole_file(const char* path) {
    FILE* f = fopen(path, "rb");
    if (!f) return NULL;
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);
    char* buf = malloc((size_t)size + 1);
    if (fread(buf, 1, (size_t)size, f) != (size_t)size) {
        fclose(f);
        free(buf);
        return NULL;
    }
    buf[size] = '\0';
    fclose(f);
    return buf;
}

/* ---- tiny modelDescription.xml scanner ----
 * Not a real XML parser -- just enough to pull `attr="value"` out of the
 * well-formed, single-line-per-attribute XML that both our own hand-written
 * modelDescription.xml files and OpenModelica's omc-generated one use.
 * Searches forward from `search_from` for `attr_name="`, copies out
 * everything up to the closing quote. Returns 0 (leaves out untouched) if
 * not found. */
static int find_attr_value(const char* search_from, const char* attr_name, char* out, size_t out_size) {
    char needle[64];
    snprintf(needle, sizeof(needle), "%s=\"", attr_name);
    const char* p = strstr(search_from, needle);
    if (!p) return 0;
    p += strlen(needle);
    const char* q = strchr(p, '"');
    if (!q) return 0;
    size_t len = (size_t)(q - p);
    if (len >= out_size) len = out_size - 1;
    memcpy(out, p, len);
    out[len] = '\0';
    return 1;
}

/* Real callback functions instead of NULL: our own hand-written FMUs
 * tolerate a NULL fmi2CallbackFunctions* (fall back to malloc), but the
 * FMI2 spec doesn't actually make these optional, and OpenModelica-
 * generated FMUs dereference them unconditionally (e.g. to log during
 * instantiation) -- passing NULL segfaults inside their fmi2Instantiate.
 * fmpy always supplies real callbacks for the same reason. */
static void fmu_logger(fmi2ComponentEnvironment env, fmi2String instanceName, fmi2Status status,
                        fmi2String category, fmi2String message, ...) {
    (void)env; (void)status;
    va_list ap;
    va_start(ap, message);
    fprintf(stderr, "[%s|%s] ", instanceName ? instanceName : "?", category ? category : "?");
    vfprintf(stderr, message, ap);
    fprintf(stderr, "\n");
    va_end(ap);
}

static void* fmu_alloc(size_t nobj, size_t size) { return calloc(nobj, size); }
static void fmu_free(void* p) { free(p); }

FmuClient* fmu_client_open(const char* fmu_dir, const char* instance_name) {
    char xml_path[1024];
    snprintf(xml_path, sizeof(xml_path), "%s/modelDescription.xml", fmu_dir);
    char* xml = read_whole_file(xml_path);
    if (!xml) {
        fprintf(stderr, "could not read %s (pass a directory with modelDescription.xml and "
                        "binaries/linux64/ in it -- unzip a .fmu first if needed)\n", xml_path);
        return NULL;
    }

    char guid[128], model_id[128];
    const char* cosim_tag = strstr(xml, "<CoSimulation");
    if (!find_attr_value(xml, "guid", guid, sizeof(guid)) ||
        !cosim_tag || !find_attr_value(cosim_tag, "modelIdentifier", model_id, sizeof(model_id))) {
        fprintf(stderr, "could not parse guid/modelIdentifier out of %s\n", xml_path);
        free(xml);
        return NULL;
    }

    char so_path[1024];
    snprintf(so_path, sizeof(so_path), "%s/binaries/linux64/%s.so", fmu_dir, model_id);
    void* handle = dlopen(so_path, RTLD_NOW);
    if (!handle) {
        fprintf(stderr, "dlopen(%s) failed: %s\n", so_path, dlerror());
        free(xml);
        return NULL;
    }

    char abs_dir[PATH_MAX];
    char resource_uri[PATH_MAX + 32] = "";
    if (realpath(fmu_dir, abs_dir)) {
        snprintf(resource_uri, sizeof(resource_uri), "file://%s/resources", abs_dir);
    }

    fmi2Instantiate_t fmi2Instantiate = (fmi2Instantiate_t)xdlsym(handle, "fmi2Instantiate");
    fmi2SetupExperiment_t fmi2SetupExperiment = (fmi2SetupExperiment_t)xdlsym(handle, "fmi2SetupExperiment");
    fmi2EnterInitializationMode_t fmi2EnterInitializationMode =
        (fmi2EnterInitializationMode_t)xdlsym(handle, "fmi2EnterInitializationMode");
    fmi2ExitInitializationMode_t fmi2ExitInitializationMode =
        (fmi2ExitInitializationMode_t)xdlsym(handle, "fmi2ExitInitializationMode");

    fmi2CallbackFunctions callbacks = {fmu_logger, fmu_alloc, fmu_free, NULL, NULL};
    fmi2Component component = fmi2Instantiate(instance_name ? instance_name : "fmu_client_instance",
                                               fmi2CoSimulation, guid, resource_uri, &callbacks, 0, 0);
    if (!component) {
        fprintf(stderr, "fmi2Instantiate returned NULL\n");
        dlclose(handle);
        free(xml);
        return NULL;
    }
    fmi2SetupExperiment(component, 0, 0.0, 0.0, 0, 0.0);
    fmi2EnterInitializationMode(component);
    fmi2ExitInitializationMode(component);

    FmuClient* client = malloc(sizeof(FmuClient));
    client->handle = handle;
    client->xml = xml;
    strncpy(client->guid, guid, sizeof(client->guid) - 1);
    client->guid[sizeof(client->guid) - 1] = '\0';
    strncpy(client->model_id, model_id, sizeof(client->model_id) - 1);
    client->model_id[sizeof(client->model_id) - 1] = '\0';
    client->component = component;
    client->fmi2FreeInstance = (fmi2FreeInstance_t)xdlsym(handle, "fmi2FreeInstance");
    client->fmi2SetReal = (fmi2SetReal_t)xdlsym(handle, "fmi2SetReal");
    client->fmi2GetReal = (fmi2GetReal_t)xdlsym(handle, "fmi2GetReal");
    client->fmi2DoStep = (fmi2DoStep_t)xdlsym(handle, "fmi2DoStep");
    client->fmi2Terminate = (fmi2Terminate_t)xdlsym(handle, "fmi2Terminate");
    return client;
}

int fmu_client_find_vr(const FmuClient* client, const char* var_name, FmuValueReference* out_vr) {
    char needle[128];
    snprintf(needle, sizeof(needle), "name=\"%s\"", var_name);
    const char* tag = strstr(client->xml, needle);
    char vr_str[32];
    if (!tag || !find_attr_value(tag, "valueReference", vr_str, sizeof(vr_str))) return 0;
    *out_vr = (FmuValueReference)atoi(vr_str);
    return 1;
}

int fmu_client_set_real(FmuClient* client, const FmuValueReference* vrs, size_t n, const double* values) {
    return client->fmi2SetReal(client->component, vrs, n, values) == fmi2OK;
}

int fmu_client_get_real(FmuClient* client, const FmuValueReference* vrs, size_t n, double* values) {
    return client->fmi2GetReal(client->component, vrs, n, values) == fmi2OK;
}

int fmu_client_do_step(FmuClient* client, double current_time, double step_size) {
    return client->fmi2DoStep(client->component, current_time, step_size, 1) == fmi2OK;
}

void fmu_client_close(FmuClient* client) {
    if (!client) return;
    client->fmi2Terminate(client->component);
    client->fmi2FreeInstance(client->component);
    dlclose(client->handle);
    free(client->xml);
    free(client);
}

const char* fmu_client_model_id(const FmuClient* client) { return client->model_id; }
const char* fmu_client_guid(const FmuClient* client) { return client->guid; }
