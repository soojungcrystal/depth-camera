cmake_minimum_required( VERSION 2.8 )

project( sample )
add_executable( sample main.cpp )
set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "sample" )

# Find Packages
find_package( PCL 1.8 REQUIRED )

if( PCL_FOUND )
  # Additional Include Directories
  include_directories( ${PCL_INCLUDE_DIRS} )

  # Preprocessor Definitions
  add_definitions( ${PCL_DEFINITIONS} )

  # Additional Library Directories
  link_directories( ${PCL_LIBRARY_DIRS} )

  # Additional Dependencies
  target_link_libraries( sample ${PCL_LIBRARIES} )
endif()