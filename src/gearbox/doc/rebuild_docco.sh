#!/bin/sh

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

if [ $# -ne 1 ]; then
  echo "Usage: $0 <user-name>"
  exit 1
fi

DOXYGENCMD=/usr/bin/doxygen
DOXYFILE=doxyfile
TARBALL=gearbox-release-doc.tar.gz
SFUSER=$1
SFSCPHOST=web.sf.net
SFSHELLHOST=shell.sf.net
SFPROJECT=gearbox
# alexm: not sure why this just changed
# SFDIR=/home/groups/g/ge/gearbox/htdocs/gearbox
SFDIR=/home/groups/g/ge/gearbox/htdocs

force ls
force $DOXYGENCMD $DOXYFILE

# direct file copy, works but is very slow
#scp -r html/* $SFUSER@$SFSCPHOST:$SFDIR/.

# with tarball
force cd html
force tar --exclude=$TARBALL -zcvf $TARBALL *
force scp $TARBALL $SFUSER@$SFSCPHOST:

# don't know how to combine this login with commands, do it manually
echo ""
echo "when connected execute this:"
echo "cd $SFDIR; mv ~/$TARBALL .; tar -zxvf $TARBALL"
echo "to quit, type 'shutdown'"
echo ""
force ssh -t $SFUSER,$SFPROJECT@$SFSHELLHOST create #"cd $SFDIR; mv ~/$TARBALL .; tar -zxvf $TARBALL"
