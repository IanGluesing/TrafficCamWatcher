include( ExternalProject )

set( boost_URL "http://sourceforge.net/projects/boost/files/boost/1.80.0/boost_1_80_0.tar.bz2" )
set( boost_SHA1 "690a2a2ed6861129828984b1d52a473d2c8393d1" )
set( boost_INSTALL ${CMAKE_CURRENT_BINARY_DIR}/thirdParty/boost )
set( boost_INCLUDE_DIR ${boost_INSTALL}/include )
set( boost_LIB_DIR ${boost_INSTALL}/lib )

ExternalProject_Add( boost
        PREFIX boost
        URL ${boost_URL}
        URL_HASH SHA1=${boost_SHA1}
        BUILD_IN_SOURCE 1
        CONFIGURE_COMMAND
        ./bootstrap.sh
        --with-libraries=filesystem
        --with-libraries=system
        --prefix=<INSTALL_DIR>
        BUILD_COMMAND
        ./b2 install link=static variant=release threading=multi runtime-link=static
        INSTALL_COMMAND ""
        INSTALL_DIR ${boost_INSTALL} )

set( Boost_LIBRARIES
        ${boost_LIB_DIR}/libboost_filesystem.a
        ${boost_LIB_DIR}/libboost_system.a)
        
message( STATUS "Boost static libs: " ${Boost_LIBRARIES} )