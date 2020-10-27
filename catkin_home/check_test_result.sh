#!/bin/bash

catkin_test_results > tmp_test_results.txt

lines_string=$(wc -l tmp_test_results.txt)

lines_array=(${lines_string// / })
if [ ${lines_array[0]} -gt 1 ]
then
    echo [CHECK_TEST_RESULT] Too many lines, it means something failed
    echo [CHECK_TEST_RESULT] resulting log:
    cat tmp_test_results.txt
    exit 1
else 
    echo [CHECK_TEST_RESULT] All good
fi
exit 0
