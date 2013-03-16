#!/bin/sh
#
# Update from repository, and rebuild the docco.
# If nothing has changed, just exit.
#
# Usage: try_rebuild_docco [--force]

force () {
    echo "$*"
    $*

    status=$?

    if [ $status -ne 0 ]; then
        echo "Problem executing '$*'"
        exit 1
    else
        echo "Executed '$*' OK"
    fi
}


forced=0
if [ $# -eq 1 ]
then
    if [ $1 = "-f" -o $1 = "--force" ]
    then
        forced=1
    fi
fi

# doxygen
DOXYGENCMD=/usr/bin/doxygen
DOXYFILE=doxyfile
#  CVS 
#UPDATECMD="cvs update"
#CHANGESIGN="'^[UP]'"
#  SVN
UPDATECMD="svn update --non-interactive"
CHANGESIGN='^[U]'
# source
PROJDIR=$HOME/svndoc/gearbox
MODULENAME=gearbox
TARBALL=$MODULENAME-head-doc.tar.gz
# destination
HEADDIR=/home/groups/g/ge/gearbox/htdocs/head

did_update=0

date

#
# Update project
#
if ! cd $PROJDIR 2>&1; then
    echo "Problem changing to directory $PROJDIR"
    exit 1
fi
pwd
if $UPDATECMD 2>&1 | egrep -e $CHANGESIGN 2>&1; then
    echo "Project has changed."
    did_update=1
fi

#
# Exit if we are not forced and nothing changed
#
if [ $did_update -eq 0 ]; then
    if [ $forced -eq 1 ]; then
        echo "Forced to rebuild docs -- even though nothing changed."
    else
        echo "No need to rebuild docs -- nothing changed."
        exit 0;
    fi
fi

#
# Rebuild docco
#
cd $PROJDIR/doc
$DOXYGENCMD $DOXYFILE
cat html/index.html | sed "s/Generated for GearBox/Generated for GearBox (on `date`)/" > temp_index.html
mv temp_index.html html/index.html

#
# Copy it to sourceforge
#
force cd html
force tar --exclude=$TARBALL -zcvf $TARBALL *
force scp $TARBALL shell.sourceforge.net:$HEADDIR/.
ssh shell.sourceforge.net "cd $HEADDIR; tar -zxvf $TARBALL"

echo "Finished $0 at: `date`"
