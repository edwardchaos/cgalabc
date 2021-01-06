// This needs to be added wherever the tinyobjloader header is included.
#define TINYOBJLOADER_IMPLEMENTATION

// Everything becomes double rather than default float
// (described in tinyobjloader readme)
#define TINYOBJLOADER_USE_DOUBLE

#include "tiny_obj_loader.h"
