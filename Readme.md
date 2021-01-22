# OpenXR C Playground

Note: Currently this application only supports Linux/X11.

This example exercises many areas of the OpenXR API.
Some parts of the API are abstracted, though the abstractions are intentionally kept simple for simple editing.

For a simpler, more straightforward example, see
https://gitlab.freedesktop.org/monado/demos/openxr-simple-example


# Running

Unless the OpenXR runtime is installed in the file system, the `XR_RUNTIME_JSON` variable has to be set for the loader to know where to look for the runtime and how the runtime is named

    XR_RUNTIME_JSON=~/monado/build/openxr_monado-dev.json

then, you should be ready to run `./openxr-playground`.

If you want to use API layers that are not installed in the default path, set the variable `XR_API_LAYER_PATH`

    XR_API_LAYER_PATH=/path/to/api_layers/

This will enable to loader to find api layers at this path and enumerate them with `xrEnumerateApiLayerProperties()`

API Layers can be enabled either with code or the loader can be told to enable an API layer with `XR_ENABLE_API_LAYERS`

    XR_ENABLE_API_LAYERS=XR_APILAYER_LUNARG_core_validation
