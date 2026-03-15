# list all signals in decimal format
add list -decimal *

#change radix to symbolic
radix -symbolic

#add wave *
add wave -recursive -depth 10 *
#add wave u_top.clk
#add wave u_top.resetn

add wave -position end  sim:/top_tb/clk
add wave -position end  sim:/top_tb/reset_n



run 10000ns

# read in stimulus
#do stim.do

# output results
write list test.lst

# quit the simulation
quit -f
