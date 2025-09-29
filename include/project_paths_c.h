#ifndef PROJECT_PATHS_C_H
#define PROJECT_PATHS_C_H

#include <stdlib.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

// C版本的项目路径获取函数
const char* get_amcl_output_path_c();

#ifdef __cplusplus
}
#endif

// C实现
const char* get_amcl_output_path_c() {
    static char path[1024];
    const char* env_root = getenv("DORA_NAV_ROOT");
    if (env_root != NULL) {
        snprintf(path, sizeof(path), "%s/build/amcl/output.pgm", env_root);
    } else {
        strncpy(path, "/home/sunny/dora_nav/build/amcl/output.pgm", sizeof(path) - 1);
    }
    return path;
}

#ifdef __cplusplus
}
#endif

#endif // PROJECT_PATHS_C_H
