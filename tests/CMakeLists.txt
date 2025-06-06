# Create the test project as a shared library (DLL) like VS does
add_library(NudgeTests SHARED
    Vector2Tests.cpp
    Matrix2Tests.cpp
    Vector3Tests.cpp
    Matrix3Tests.cpp
    Vector4Tests.cpp
    Matrix4Tests.cpp
    MathFTests.cpp
    QuaternionTests.cpp
)

# Always link to nudge library
target_link_libraries(NudgeTests PRIVATE nudge)

# Try to find MS Unit Test Framework
if(MSVC)
    # Common Visual Studio paths - adjust version as needed
    set(VS_VERSIONS "2022" "2019" "2017")
    set(VS_EDITIONS "Professional" "Community" "Enterprise")
    
    foreach(VERSION ${VS_VERSIONS})
        foreach(EDITION ${VS_EDITIONS})
            set(MSTEST_BASE_PATH "C:/Program Files/Microsoft Visual Studio/${VERSION}/${EDITION}/VC/Auxiliary/VS/UnitTest")
            
            if(EXISTS "${MSTEST_BASE_PATH}/include" AND EXISTS "${MSTEST_BASE_PATH}/lib")
                set(MSTEST_INCLUDE_DIR "${MSTEST_BASE_PATH}/include")
                
                # Check for x64 lib first, then x86
                if(CMAKE_SIZEOF_VOID_P EQUAL 8)
                    set(MSTEST_LIB_DIR "${MSTEST_BASE_PATH}/lib")
                    # Also check x64 subdirectory
                    if(EXISTS "${MSTEST_BASE_PATH}/lib/x64")
                        set(MSTEST_LIB_DIR "${MSTEST_BASE_PATH}/lib/x64")
                    endif()
                else()
                    set(MSTEST_LIB_DIR "${MSTEST_BASE_PATH}/lib")
                endif()
                
                # Check for the actual library file
                set(MSTEST_LIB_FILE "${MSTEST_LIB_DIR}/Microsoft.VisualStudio.TestTools.CppUnitTestFramework.lib")
                if(EXISTS "${MSTEST_LIB_FILE}")
                    message(STATUS "Found MS Unit Test Framework at: ${MSTEST_BASE_PATH}")
                    message(STATUS "Library file: ${MSTEST_LIB_FILE}")
                    break()
                endif()
            endif()
        endforeach()
        if(MSTEST_INCLUDE_DIR)
            break()
        endif()
    endforeach()
    
    if(MSTEST_INCLUDE_DIR AND MSTEST_LIB_DIR)
        target_include_directories(NudgeTests PRIVATE
            ${MSTEST_INCLUDE_DIR}
        )
        
        target_link_libraries(NudgeTests PRIVATE
            ${MSTEST_LIB_DIR}/Microsoft.VisualStudio.TestTools.CppUnitTestFramework.lib
        )
        
        # Add required compile definitions for MS Unit Test DLL
        target_compile_definitions(NudgeTests PRIVATE
            _UNICODE
            UNICODE
            _WINDLL
        )
        
        message(STATUS "MS Unit Test Framework linked successfully")
        message(STATUS "Include dir: ${MSTEST_INCLUDE_DIR}")
        message(STATUS "Lib dir: ${MSTEST_LIB_DIR}")
        message(STATUS "Lib file: ${MSTEST_LIB_FILE}")
        
        # Set properties for test DLL
        if(WIN32)
            set_target_properties(NudgeTests PROPERTIES
                SUFFIX ".dll"
                RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tests"
            )
        endif()
    else()
        message(WARNING "MS Unit Test Framework not found. Test will compile but may not link properly.")
        message(STATUS "Searched paths:")
        foreach(VERSION ${VS_VERSIONS})
            foreach(EDITION ${VS_EDITIONS})
                message(STATUS "  C:/Program Files/Microsoft Visual Studio/${VERSION}/${EDITION}/VC/Auxiliary/VS/UnitTest")
            endforeach()
        endforeach()
    endif()
else()
    message(WARNING "MS Unit Test Framework is only available on Windows with MSVC")
endif()

# Add the test DLL to CTest (Visual Studio Test Explorer will discover it automatically)
# Note: CTest registration is optional for VS Test Explorer discovery
if(MSTEST_INCLUDE_DIR AND MSTEST_LIB_DIR)
    # Register with CTest if desired
    add_test(NAME NudgeTests 
             COMMAND ${CMAKE_COMMAND} -E echo "NudgeTests.dll built - run via Test Explorer")
endif()