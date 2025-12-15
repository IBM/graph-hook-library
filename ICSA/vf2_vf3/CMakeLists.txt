# Copyright (c) 2025 IBM
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

function(add_benchmark name)
    add_executable(${name} ${ARGN})
    foreach(boost_lib ${BOOST_INCLUDE_LIBRARIES})
        if(TARGET Boost::${boost_lib})
            target_link_libraries(${name} PRIVATE Boost::${boost_lib})
        endif()
    endforeach()
endfunction()

file(GLOB BENCHMARK_SOURCES "*.cpp")
foreach(benchmark_file ${BENCHMARK_SOURCES})
    get_filename_component(benchmark_name ${benchmark_file} NAME_WE)
    add_benchmark(${benchmark_name} ${benchmark_file})
endforeach()

add_custom_command(
    TARGET vf3_vs_vf2 POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
    $<TARGET_FILE:vf3_vs_vf2>
    ${CMAKE_CURRENT_SOURCE_DIR}/vf3_vs_vf2
    COMMENT "Copying vf3_vs_vf2 executable to benchmark sources"
)
