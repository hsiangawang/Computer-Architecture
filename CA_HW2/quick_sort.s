.data
str1: .asciiz   "Quick:Input the size: \n"
str2: .asciiz   "Input the number: \n"
str3: .asciiz   "\nResult: "
space: .asciiz  " "

.text
       main:li $v0, 4
			la $a0, str1
			syscall				

			li $v0, 5			#input size
			syscall
			move $t9, $v0      #把size存到t9
			move $t0, $v0      #把size存到t0
			move $s0, $v0       #s0 is size
	
    		sll $t1, $t0, 3   #size*8放到t1
    		sll $t3, $t0, 2   #size*4放到t3
			move $a0, $t1      #把t1放到a0，為了留記憶體

			li $v0, 9          #allocate memory
			syscall				
    		move $s4, $v0	   #存地址到s4，之後拿來存起始地址
			add $a1, $v0, $t3     #把這個地址放到a1
	
			li $v0, 4          
			la $a0, str2
			syscall
    
      input:beq $s1, $s0, allocate	#s1等於size的話就跳去allocate
    
    		li $v0, 5               
			syscall

			sll $t0, $s1, 2          #把s1*4之後放到t0 
			add $t1, $s4, $t0       #t1=data[i]
	
			sw $v0, 0($t1)          #把輸進來的值放到data[i]

			addi $s1, $s1, 1        #i=i+1
   			 j input

     output:move $s1,$zero         #把i歸零
	
			li $v0, 4
			la $a0, str3
			syscall

     print:	beq $s1, $t9, return    #if i=size , stop
    		sll $t0, $s1, 2         #t0=i*4
			add $t1, $a1, $t0       #t1=data[i]
			lw $a0, 0($t1)          #load data[i] to a0

			li $v0, 1               #print
			syscall

			addi $s1, $s1, 1        #i=i+1

			li $v0, 4
			la $a0, space
			syscall
    		j print

     return:li $v0, 10
			syscall

recursive:

   allocate:addi $s1, $zero, 1 	#initiate to 1, used as pivot
	
			move $s3,$zero     #record the size of great_list
			move $s5,$zero     #record the size of small_list
			move $s2,$s0	   #let s2 also the size	

			sll $t0, $s0, 3    #t0=size*8
			sll $a3, $s0, 2	   #a3=size*4	
			move $a0,$t0       #a0=size*8

			li $v0, 9          #要兩塊跟size一樣大的記憶體
			syscall

			move $s7,$v0		#把這個地址放到s7
			add $s6, $v0, $a3   #另一個放到s6  

    		lw $t0,0($s4)

  partition:beq $s1, $s2, small_recur  #j=size的話就去下一層

			sll $t5, $s1, 2  		   #t5=j*4	
			add $t6, $t5, $s4          #t6=data[j]
			lw $t1, 0($t6)             #load t1 =data[j] 

			slt $t6, $t1, $t0          #if data[j]>data[j-1],t6=0 ,go to great_list
			beq $t6, $zero, great_list #go to great_list and put it in

 small_list:sll $t5, $s5, 2            #s5(small list's size)*4＝t5
			add $t4, $t5, $s7	       #s7(small_list)的地址+t5=small_list[size](t4)
			sw  $t1, 0($t4)            #把t1(data[j])放到small_list[size](t4)裡
			addi $s5, $s5, 1		   #small_list size+1	
			addi $s1, $s1, 1           #j=j+1
			j partition

great_list:sll $t5, $s3, 2            #if data[j]>data[j-1]  s3(great_list size)*4
			add $t4, $t5, $s6	       #s6(great_list)address+j*4=t4
			sw $t1, 0($t4)			   #t1(data[j]) put into great_list[size](t4)	

			addi $s3, $s3, 1           #great_list size+1
			addi $s1, $s1, 1		   #j=j+1	
			j partition


small_recur:beq $s5, $zero, great_recur  #如果small_list裡沒東西，去great_recur
	        addi $t1, $zero, 1           #let t1 be 1
			bne $s5, $t1, store_small    #if small_list裡有不只一個東西，把東西存起來

    		#如果small_list只有一個
			lw $t0, 0($s7)  #load small_list的唯一一個到t0

			sll $t2, $t7, 2   #t7=i,i*4
			add $t2, $a1, $t2  #a1(sorting好的放的位置)+i*4=t2

			sw $t0, 0($t2)	#把t0(small_list的唯一一個)放到a3[i]

			addi $t7, $t7, 1 #i=i+1

			j great_recur

store_small:addi $sp, $sp, -36
			sw $ra, 0($sp)
			sw $s7, 4($sp)
			sw $s6, 8($sp)
			sw $s5, 12($sp)
			sw $s4, 16($sp)
			sw $s3, 20($sp)
			sw $s2, 24($sp)
			sw $s1, 28($sp)
			sw $s0, 32($sp)

			move $s4,$s7           #s4(input address已經存起來了)，接下來把s7(small_size address放到input address)，這樣做recurrence
			move $s0,$s5           #small list size 則放到 s0

			jal recursive
	
			lw $ra, 0($sp)
			lw $s7, 4($sp)
			lw $s6, 8($sp)
			lw $s5, 12($sp)
			lw $s4, 16($sp)
			lw $s3, 20($sp)
			sw $s2, 24($sp)
			sw $s1, 28($sp)
			sw $s0, 32($sp)
			addi $sp, $sp, 36

great_recur:lw $t1, 0($s4)          #把input第一個值放到t1
			sll $t2, $t7, 2         #t2=i*4
			add $t2, $a1, $t2       #a1(sorting好的放的位置)+i*4=t2
			sw $t1, 0($t2)		    #把t1(input第一個值)放到a1[i]
    		addi $t7, $t7, 1        #i=i+1

			beq $s3, $zero, check   #當great_list size=0,表示已經sorting好了
			li $t1,1      #let t1=1
			bne $s3, $t1, store_great  #如果great_list有一個以上的東西，存起來遞迴

			lw $t0, 0($s6)   #great_list只有一個
			sll $t2, $t7, 2  #t2=i*4
			add $t2, $a1, $t2 #a1(sorting好的放的位置)+i*4=t2
			sw $t0, 0($t2) #把t0(small_list的唯一一個)放到a1[i]

			addi $t7, $t7, 1 ##i=i+1
			j check

store_great:addi $sp, $sp, -36
			sw $ra, 0($sp)
			sw $s7, 4($sp)
			sw $s6, 8($sp)
			sw $s5, 12($sp)
			sw $s4, 16($sp)
			sw $s3, 20($sp)
			sw $s2, 24($sp)
			sw $s1, 28($sp)
			sw $s0, 32($sp)

			move $s4,$s6  #s4(input address已經存起來了)，接下來把s6(great_size address放到input address)，這樣做recurrence
			move $s0,$s3  #great list size 則放到 s0

			jal recursive

			lw $ra, 0($sp)
			lw $s7, 4($sp)
			lw $s6, 8($sp)
			lw $s5, 12($sp)
			lw $s4, 16($sp)
			lw $s3, 20($sp)
			sw $s2, 24($sp)
			sw $s1, 28($sp)
			sw $s0, 32($sp)
			addi $sp, $sp, 36

      check:add $t1, $s5, $s3  
	  		addi $t1, $t1, 1
			beq $t1, $t9, output
			jr $ra
