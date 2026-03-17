#!/bin/bash
echo run tests
echo


cd test0_axi_avalon_bridge
echo "test0_axi_avalon_bridge"
result=$(./simulate.sh)
if echo "$result" | grep "PASS"; then
    printf "PASS!\n"
elif echo "$result" | grep "FAIL"; then
    printf "FAIL!\n"
    exit 1
else
    printf "Unknown result\n"
    exit 1
fi
echo ""
cd ..


cd test0_axi_avalon_bridge
echo "test1_sdram_controller"
result=$(./simulate.sh)
if echo "$result" | grep "PASS"; then
    printf "PASS!\n"
elif echo "$result" | grep "FAIL"; then
    printf "FAIL!\n"
    exit 1
else
    printf "Unknown result\n"
    exit 1
fi
echo ""
cd ..
