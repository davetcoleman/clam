# generated from genmsg/cmake/pkg-genmsg.cmake.em

@{
import os
import sys

import genmsg
import genmsg.base
genmsg.base.log_verbose('GENMSG_VERBOSE' in os.environ)
import genmsg.deps
import genmsg.gentools

# split incoming variables
messages = messages_str.split(';') if messages_str != '' else []
services = services_str.split(';') if services_str != '' else []
dependencies = dependencies_str.split(';') if dependencies_str != '' else []
dep_search_paths = dep_include_paths_str.split(';') if dep_include_paths_str != '' else []

dep_search_paths_dict = {}
dep_search_paths_tuple_list = []
is_even = True
for val in dep_search_paths:
    if is_even:
        dep_search_paths_dict.setdefault(val, [])
        val_prev = val
        is_even = False
    else:
        dep_search_paths_dict[val_prev].append(val)
        dep_search_paths_tuple_list.append((val_prev, val))
        is_even = True
dep_search_paths = dep_search_paths_dict

msg_deps = {}
for m in messages:
  msg_deps[m] = genmsg.deps.find_msg_dependencies(pkg_name, m, dep_search_paths)

srv_deps = {}
for s in services:
  srv_deps[s] = genmsg.deps.find_srv_dependencies(pkg_name, s, dep_search_paths)

}@
message(STATUS "@(pkg_name): @(len(messages)) messages, @(len(services)) services")

set(MSG_I_FLAGS "@(';'.join(["-I%s:%s" % (dep, dir) for dep, dir in dep_search_paths_tuple_list]))")

# Find all generators
@[if langs]@
@[for l in langs.split(';')]@
find_package(@l REQUIRED)
@[end for]@
@[end if]@

#better way to handle this?
set (ALL_GEN_OUTPUT_FILES_cpp "")

#
#  langs = @langs
#

@[if langs]@
@[for l in langs.split(';')]@
### Section generating for lang: @l
### Generating Messages
@[for m in messages]@
_generate_msg_@(l[3:])(@pkg_name
  @m
  "${MSG_I_FLAGS}"
  "@(';'.join(msg_deps[m]).replace("\\","/"))"
  ${CATKIN_DEVEL_PREFIX}/${@(l)_INSTALL_DIR}/@pkg_name
)
@[end for]@# messages

### Generating Services
@[for s in services]@
_generate_srv_@(l[3:])(@pkg_name
  @s
  "${MSG_I_FLAGS}"
  "@(';'.join(srv_deps[s]).replace("\\","/"))"
  ${CATKIN_DEVEL_PREFIX}/${@(l)_INSTALL_DIR}/@pkg_name
)
@[end for]@# services

### Generating Module File
_generate_module_@(l[3:])(@pkg_name
  ${CATKIN_DEVEL_PREFIX}/${@(l)_INSTALL_DIR}/@pkg_name
  "${ALL_GEN_OUTPUT_FILES_@(l[3:])}"
)

add_custom_target(@(pkg_name)_@(l) ALL
  DEPENDS ${ALL_GEN_OUTPUT_FILES_@(l[3:])}
)

@[end for]@# langs
@[end if]@

debug_message(2 "@pkg_name: Iflags=${MSG_I_FLAGS}")

@[if langs]@
@[for l in langs.split(';')]@

if(@(l)_INSTALL_DIR)
@[if l == 'genpy']@
  install(CODE "execute_process(COMMAND \"@(PYTHON_EXECUTABLE)\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${@(l)_INSTALL_DIR}/@pkg_name\")")
@[end if]@
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${@(l)_INSTALL_DIR}/@pkg_name
    DESTINATION ${@(l)_INSTALL_DIR}
@[if l == 'genpy' and package_has_static_sources]@
    # skip all init files
    PATTERN "__init__.py" EXCLUDE
    PATTERN "__init__.pyc" EXCLUDE
  )
  # install init files which are not in the root folder of the generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${@(l)_INSTALL_DIR}/@pkg_name
    DESTINATION ${@(l)_INSTALL_DIR}
    FILES_MATCHING
    REGEX "/@(pkg_name)/.+/__init__.pyc?$"
@[end if]@
  )
endif()
@[for d in dependencies]@
add_dependencies(@(pkg_name)_@(l) @(d)_@(l))
@[end for]@# dependencies
@[end for]@# langs
@[end if]@
