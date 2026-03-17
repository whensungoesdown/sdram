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



#run 20000ns

#
## 2. 如果还停在初始化，手动强制
#force -freeze u_sdram_ctrl.init_done 1
#force -freeze u_sdram_ctrl.i_state 3'b101
#force -freeze u_sdram_ctrl.refresh_request 0

# 3. 继续运行
run -all

# read in stimulus
#do stim.do

# output results
write list test.lst

# quit the simulation
quit -f
