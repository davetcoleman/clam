# This script is indended for Gearbox users (not for internal use)
# Assumes that the following variables are already defined
# (typically by first loading gearbox-config.cmake)
# - GEARBOX_INCLUDE_DIR
# - GEARBOX_LINK_DIR

include_directories( ${GEARBOX_INCLUDE_DIR} )

# Variable GEARBOX_LINK_DIR only needs to be defined if you're using a gearbox
# installation (as opposed to a local gearbox repository)
link_directories( ${GEARBOX_LINK_DIR} )

# Not linking to specific libraries, it's up to the user which ones to link to.
