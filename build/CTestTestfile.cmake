# CMake generated Testfile for 
# Source directory: /home/mia/workspace/pc2mesh
# Build directory: /home/mia/workspace/pc2mesh/build
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_enhanced_logging_system "/home/mia/workspace/pc2mesh/build/bin/test_enhanced_logging_system")
set_tests_properties(test_enhanced_logging_system PROPERTIES  _BACKTRACE_TRIPLES "/home/mia/workspace/pc2mesh/CMakeLists.txt;445;add_test;/home/mia/workspace/pc2mesh/CMakeLists.txt;0;")
add_test(test_complete_system_integration "/home/mia/workspace/pc2mesh/build/bin/test_complete_system_integration")
set_tests_properties(test_complete_system_integration PROPERTIES  _BACKTRACE_TRIPLES "/home/mia/workspace/pc2mesh/CMakeLists.txt;445;add_test;/home/mia/workspace/pc2mesh/CMakeLists.txt;0;")
add_test(test_thread_safe_solver "/home/mia/workspace/pc2mesh/build/bin/test_thread_safe_solver")
set_tests_properties(test_thread_safe_solver PROPERTIES  _BACKTRACE_TRIPLES "/home/mia/workspace/pc2mesh/CMakeLists.txt;445;add_test;/home/mia/workspace/pc2mesh/CMakeLists.txt;0;")
add_test(test_optimized_udf_simple "/home/mia/workspace/pc2mesh/build/bin/test_optimized_udf_simple")
set_tests_properties(test_optimized_udf_simple PROPERTIES  _BACKTRACE_TRIPLES "/home/mia/workspace/pc2mesh/CMakeLists.txt;445;add_test;/home/mia/workspace/pc2mesh/CMakeLists.txt;0;")
add_test(test_enhanced_dual_contouring_simple "/home/mia/workspace/pc2mesh/build/bin/test_enhanced_dual_contouring_simple")
set_tests_properties(test_enhanced_dual_contouring_simple PROPERTIES  _BACKTRACE_TRIPLES "/home/mia/workspace/pc2mesh/CMakeLists.txt;445;add_test;/home/mia/workspace/pc2mesh/CMakeLists.txt;0;")
add_test(test_enhanced_detail_reconstruction_simple "/home/mia/workspace/pc2mesh/build/bin/test_enhanced_detail_reconstruction_simple")
set_tests_properties(test_enhanced_detail_reconstruction_simple PROPERTIES  _BACKTRACE_TRIPLES "/home/mia/workspace/pc2mesh/CMakeLists.txt;445;add_test;/home/mia/workspace/pc2mesh/CMakeLists.txt;0;")
add_test(test_enhanced_mesh_fusion_simple "/home/mia/workspace/pc2mesh/build/bin/test_enhanced_mesh_fusion_simple")
set_tests_properties(test_enhanced_mesh_fusion_simple PROPERTIES  _BACKTRACE_TRIPLES "/home/mia/workspace/pc2mesh/CMakeLists.txt;445;add_test;/home/mia/workspace/pc2mesh/CMakeLists.txt;0;")
add_test(integration_test "/home/mia/workspace/pc2mesh/build/bin/integration_test")
set_tests_properties(integration_test PROPERTIES  _BACKTRACE_TRIPLES "/home/mia/workspace/pc2mesh/CMakeLists.txt;445;add_test;/home/mia/workspace/pc2mesh/CMakeLists.txt;0;")
add_test(minimal_test "/home/mia/workspace/pc2mesh/build/bin/minimal_test")
set_tests_properties(minimal_test PROPERTIES  _BACKTRACE_TRIPLES "/home/mia/workspace/pc2mesh/CMakeLists.txt;445;add_test;/home/mia/workspace/pc2mesh/CMakeLists.txt;0;")
add_test(performance_test "/home/mia/workspace/pc2mesh/build/bin/test_complete_system_integration")
set_tests_properties(performance_test PROPERTIES  TIMEOUT "300" _BACKTRACE_TRIPLES "/home/mia/workspace/pc2mesh/CMakeLists.txt;451;add_test;/home/mia/workspace/pc2mesh/CMakeLists.txt;0;")
