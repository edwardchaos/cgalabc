set (BINARY ${CMAKE_PROJECT_NAME}_test)

# specify source files
set(TEST_SOURCES
	TestLine2d.cpp
	TestAlgo2d.cpp
)
set(TEST_HEADERS

)

# create test executable
add_executable(${BINARY} ${TEST_SOURCES} ${TEST_HEADERS})

add_test(NAME ${BINARY}
		 COMMAND ${BINARY})

# link with cgalabc library
target_link_libraries(${BINARY} PUBLIC ${CMAKE_PROJECT_NAME}_lib gtest_main)
