# Copyright 1999-2007 Gentoo Foundation
# Distributed under the terms of the GNU General Public License v2
# $Header: $

inherit cmake-utils

SLOT="0"
LICENSE="GPL-2"
KEYWORDS="~x86 ~amd64"
DESCRIPTION="A collection of libraries for robotics, including hardware drivers and algorithms."
SRC_URI="/${P}.tar.bz2"
HOMEPAGE="http://gearbox.sourceforge.net"
IUSE="doc basic gbxadvanced gbxgarminacfr gbxserialacfr gbxsickacfr gbxsmartbatteryacfr gbxutilacfr hokuyo_aist"
DEPEND=">=dev-util/cmake-2.4
		doc? (app-doc/doxygen)"
RDEPEND=${DEPEND}

S=${WORKDIR}/${PN}

src_compile()
{
	local mycmakeargs="`cmake-utils_use_enable basic LIB_BASIC`\
						`cmake-utils_use_enable gbxadvanced LIB_GBXADVANCED`\
						`cmake-utils_use_enable gbxgarminacfr LIB_GBXGARMINACFR`\
						`cmake-utils_use_enable gbxserialacfr LIB_GBXSERIALACFR`\
						`cmake-utils_use_enable gbxsickacfr LIB_GBXSICKACFR`\
						`cmake-utils_use_enable gbxsmartbatteryacfr LIB_GBXSMARTBATTERYACFR`\
						`cmake-utils_use_enable gbxutilacfr LIB_GBXUTILACFR`\
						`cmake-utils_use_enable hokuyo_aist LIB_HOKUYO_AIST`"
	cmake-utils_src_compile
	if use doc;
	then
		cd doc
		doxygen doxyfile || die "Error building documentation"
	fi
}

src_install ()
{
	cmake-utils_src_install
	dodoc LICENSE
	use doc && dohtml -r doc/html/*
}