#!/bin/bash
let "SUCCESS_COUNT=0"
while ./gradlew cleanTest test --info && (($SUCCESS_COUNT < 100))
do
    let "SUCCESS_COUNT++"
    echo "Success Count: $SUCCESS_COUNT"
done
