.data

str1: .asciiz   "Bubble:Input the size: \n"
str2: .asciiz   "Input the number: \n"
str3: .asciiz   "\nResult: "
space:.asciiz   " "
 


.text 

       main:li $v0,4       #print string "Input the size \n"
            la $a0,str1    
            syscall
           
            li $v0,5 
            syscall 
            move $t5,$v0 # t5 is the size    
            move $a1,$v0       

                             
            move $t6,$t5
            sll  $t6, $t6, 2
            move $a0,$t6
            li $v0,9
            syscall

            move $t8,$v0
            move $t7,$v0
            move $t6,$v0

            
            li $v0,4       #print string "Input the number \n"
            la $a0,str2    
            syscall

            move $t0,$zero
            move $t1,$zero

      input:li $v0,5
            syscall
            sll $t0, $t1, 2
            add $t8, $t7, $t0
            sw $v0, 0($t8)
            addi $t1, $t1, 1
            bne $t1, $t5, input

            move $t0,$zero
            move $t1,$zero
           
                
            li $v0,4     #print string "\nResult: "
            la $a0,str3
            syscall

            move $a0,$t7

            jal bubble_sort

            
            add $s7, $s7, 0

      print:

            sll $t9, $s7, 2
            add $t6, $t7, $t9
            

            li $v0,1      #print integer
            lw $a0, 0($t6)
            syscall

            li $v0,4;                # print string
            la $a0,space;            # space
            syscall;                 # call

            addi $s7, $s7, 1

            bne $s7, $t5, print

            li $v0, 10
            syscall


bubble_sort:addi $sp,$sp,-20 #make room on stack for five register
            sw $ra, 16($sp)  #save ra on stack
            sw $s3, 12($sp)  #save s3 on stack
            sw $s2, 8($sp)   #save s2 on stack 
            sw $s1, 4($sp)   #save s1 on stack
            sw $s0, 0($sp)    #save s0 on stack

            move $s2,$a0 #save a0(v),so now v is in s2  
            move $s3,$a1 #save a1(n),so now n is in s3
            #outer loop,do initiate before the label 
            move $s0,$zero #initiate
    for1tst:slt $t0, $s0, $s3 #if i<n,then t0 will be 1
            beq $t0,$zero,exit1 #if equal is true,means i>n
            addi $s1, $s0, -1 #j=i-1

    for2tst:slti $t0, $s1, 0 #if j<0 ,t0 will be 1
            bne $t0, $zero, exit2 # if not equal is true , j<0
            sll $t1, $s1 , 2 #t1 = j*4,in order to be the address
            add $t2, $s2, $t1 #t2=v+j*4,get the address
            lw $t3, 0($t2) #t3=v[j]
            lw $t4, 4($t2) #t4=v[j+1]
            slt $t0, $t4, $t3 #if v[j+1]>v[j],t0 will be 0
            beq $t0, $zero, exit2 #go to exit2 if v[j+1]>v[j]

            move $a0,$s2 #first parameter of swap is v
            move $a1,$s1 #second parameter of swap is j
            jal swap #swap

            addi $s1, $s1, -1 #j=j-1
            j for2tst
      exit2:addi $s0, $s0, 1 #i=i+1
            j for1tst



      exit1:lw $s0, 0($sp) #restore s0 from stack
            lw $s1, 4($sp) #restore s1 from stack
            lw $s2, 8($sp) #restore s2 from stack
            lw $s3, 12($sp) #restore s3 from stack
            lw $ra, 16($sp) #restore ra from stack
            addi $sp, $sp, 20 #restore stack pointer

            jr $ra #return to calling routine

       swap:sll $t1, $a1, 2 #t1=j*4
            add $t1, $a0, $t1 #t1=v+j*4

            lw $t0, 0($t1) #t0=v[j]
            lw $t2, 4($t1) #t2=v[j+1]

            sw $t2, 0($t1) #v[j]=t2
            sw $t0, 4($t1) #v[j+1]=t0

            jr $ra




           
      








