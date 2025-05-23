include(InstallRequiredSystemLibraries)

# set general information for package
set(CPACK_PACKAGING_INSTALL_PREFIX "/opt/xbot" CACHE PATH "Deb package install prefix")
set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_NAME ${PROJECT_NAME})
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "amd64")
set(CPACK_PACKAGE_ARCHITECTURE "amd64")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "arturo.laurenzi@iit.it")
set(CPACK_DEBIAN_PACKAGE_DESCRIPTION "Iit's centauro description and gazebo simulator")

set(CPACK_DEBIAN_PACKAGE_DEPENDS "")
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)
set(CPACK_DEBIAN_PACKAGE_GENERATE_SHLIBS ON)

# get short sha1 of current commit from git
find_program(GIT_SCM git DOC "Git version control")
mark_as_advanced(GIT_SCM)
find_file(GITDIR NAMES .git PATHS ${CMAKE_SOURCE_DIR} NO_DEFAULT_PATH)
if (GIT_SCM AND GITDIR)
    execute_process(
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMAND ${GIT_SCM} log -1 "--pretty=format:%h"
        OUTPUT_VARIABLE GIT_SHA1_SHORT
    )
endif()

# get linux distro codename
execute_process(
    COMMAND lsb_release -cs
    OUTPUT_VARIABLE LINUX_DISTRO_NAME
)

string(REPLACE "\n" "" LINUX_DISTRO_NAME ${LINUX_DISTRO_NAME})

# set filename and version
set(CPACK_PACKAGE_FILE_NAME ${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}-${GIT_SHA1_SHORT}-${CPACK_PACKAGE_ARCHITECTURE}-${LINUX_DISTRO_NAME})
set(CPACK_DEBIAN_PACKAGE_VERSION "${CPACK_PACKAGE_VERSION}-${GIT_SHA1_SHORT}-${CPACK_PACKAGE_ARCHITECTURE}-${LINUX_DISTRO_NAME}")

# info message
message(STATUS "Will generate package '${CPACK_PACKAGE_FILE_NAME}.deb'")

include(CPack)
