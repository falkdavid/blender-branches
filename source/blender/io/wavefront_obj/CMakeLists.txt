# SPDX-License-Identifier: GPL-2.0-or-later

set(INC
  .
  ./exporter
  ./importer
  ../../blenkernel
  ../../blenlib
  ../../bmesh
  ../../bmesh/intern
  ../../depsgraph
  ../../editors/include
  ../../makesdna
  ../../makesrna
  ../../nodes
  ../../windowmanager
  ../../../../extern/fmtlib/include
  ../../../../intern/guardedalloc
)

set(INC_SYS

)

set(SRC
  IO_wavefront_obj.cc
  exporter/obj_export_file_writer.cc
  exporter/obj_export_mesh.cc
  exporter/obj_export_mtl.cc
  exporter/obj_export_nurbs.cc
  exporter/obj_exporter.cc
  importer/importer_mesh_utils.cc
  importer/obj_import_file_reader.cc
  importer/obj_import_mesh.cc
  importer/obj_import_mtl.cc
  importer/obj_import_nurbs.cc
  importer/obj_importer.cc
  importer/parser_string_utils.cc

  IO_wavefront_obj.h
  exporter/obj_export_file_writer.hh
  exporter/obj_export_io.hh
  exporter/obj_export_mesh.hh
  exporter/obj_export_mtl.hh
  exporter/obj_export_nurbs.hh
  exporter/obj_exporter.hh
  importer/importer_mesh_utils.hh
  importer/obj_import_file_reader.hh
  importer/obj_import_mesh.hh
  importer/obj_import_mtl.hh
  importer/obj_import_nurbs.hh
  importer/obj_import_objects.hh
  importer/obj_importer.hh
  importer/parser_string_utils.hh
)

set(LIB
  bf_blenkernel
)

if(WITH_TBB)
  add_definitions(-DWITH_TBB)
  list(APPEND INC_SYS ${TBB_INCLUDE_DIRS})
  list(APPEND LIB ${TBB_LIBRARIES})
endif()

blender_add_lib(bf_wavefront_obj "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")

if(WITH_GTESTS)
  set(TEST_SRC
    tests/obj_exporter_tests.cc
    tests/obj_importer_tests.cc
    tests/obj_exporter_tests.hh
  )

  set(TEST_INC
    ${INC}

    ../../blenloader
    ../../../../tests/gtests
  )

  set(TEST_LIB
    ${LIB}

    bf_blenloader_tests
    bf_wavefront_obj
  )

  include(GTestTesting)
  blender_add_test_lib(bf_wavefront_obj_tests "${TEST_SRC}" "${TEST_INC}" "${INC_SYS}" "${TEST_LIB}")
  add_dependencies(bf_wavefront_obj_tests bf_wavefront_obj)
endif()
