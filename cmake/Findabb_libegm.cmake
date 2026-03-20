find_library(abb_libegm_LIBRARY NAMES abb_libegm PATH_SUFFIXES lib)
find_path(abb_libegm_INCLUDE_DIR NAMES egm_common.h PATH_SUFFIXES include/abb_libegm)

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(abb_libegm
    FOUND_VAR abb_libegm_FOUND
    REQUIRED_VARS abb_libegm_LIBRARY abb_libegm_INCLUDE_DIR)

include(CMakeFindDependencyMacro)
find_dependency(Protobuf)
find_package(Boost COMPONENTS system thread)

if(abb_libegm_FOUND)
    set(abb_libegm_LIBRARIES ${abb_libegm_LIBRARY} ${PROTOBUF_LIBRARIES} ${Boost_LIBRARIES})
    set(abb_libegm_INCLUDE_DIRS ${abb_libegm_INCLUDE_DIR} ${Boost_INCLUDE_DIRS})
endif()

mark_as_advanced(abb_libegm_LIBRARY abb_libegm_INCLUDE_DIR)
