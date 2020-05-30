#include<iostream>
#include<stdio.h>
#include<stdlib.h>
#include "L1cache.h"
#include "Mem.h"
using namespace std;

L1cache::L1cache (Mem* memory,int way = 1) {
	L1readhit = 0;
	L1readmiss = 0;
	L1writehit = 0;
	L1writemiss = 0;
	
	mem = memory;
	way_number = way;
	for(int i=0; i < L1size; i++) {
		for (int j=0; j < 4; j++) {
			cache[i][j] = 0;
		}
	}
}

int L1cache::getfromCache(const int address){

	//==================================================
	cout<<"printing cache content.....  (load address : "<<address<<')'<<endl;
	for(int i=0; i<L1size/2; i++)
	{
		cout<<i<<" : ";
		for(int j=0; j<8; j++)
			cout<<"["<<cache[i][j]<<"]";
		cout<<"  \t"<<i+4<<" : ";
		for(int j=0; j<8; j++)
			cout<<"["<<cache[i+4][j]<<"]";
		cout<<endl;
	}

	int data_allocate = address%4;      //to get data in which byte(0~3)
	int off_set = address / 4;			// 給的address要先拿掉後面兩個bit
	int set_total=L1size/way_number; //把size/way可以得到到底有幾個set
	int index = off_set %set_total;   //要從address那拿到index 就把它mod settotal
	int block_offset = index*way_number; //index乘一個set有多少就是第幾個index的offset
	int address_tag = off_set /set_total; //從address拿到tag
	int counter = 0;
	int replace=0;
    
	for(int i = block_offset; i<block_offset + way_number; i++)   // Here are cache readhit! and will return the data in cache allocate
	{
		if((cache[i][0] == 0)||(cache[i][3] != address_tag))  // read's requirement
			continue;
		else 
		{
			L1readhit++;
			counter++;
			cache[i][2]=counter;
			return cache[i][4 +data_allocate]; //return the value in cache
		}
	}
	//從這裡開始都是miss

	L1readmiss++;

	for(int i = block_offset; i < block_offset + way_number; i++)
	{
		if(cache[i][0] == 1)
			continue; //表示有放東西
		else
		{
			
			cache[i][0] = 1;
			cache[i][1] = 0;
			for (int j = 0; j < 4; j++)
			{
				int* addressPTR = mem->getfromMem(off_set);
				cache[i][4 + j] = addressPTR[j];
			}
			counter++;
			cache[i][2] = counter;
			cache[i][3] = address_tag;
			return cache[i][4 + data_allocate]; //回傳對應的value (可能寫錯)
		}
	}


	//from here all the blocks are full , we need to determine which one is the LRU and writeback


	for (int i = 0; i < way_number;i++)
	{
		int temp=cache[block_offset][2];
		if (cache[block_offset+i][2]>=temp)
			continue;
		else
		{
			temp=cache[block_offset+i][2];
			replace=i;
		}

	} //then we can get which block need to be replace;



	if (cache[block_offset+replace][1] == 1)  //表示被更新過值 writeback
		{
			int WBaddress= cache[block_offset+replace][3]*set_total+index; //還原回原本的address
			int* WBPTR = new int[4];
				for(int j=0; j<4; j++)
					WBPTR[j] = cache[block_offset+replace][4+j];
					mem->writetoMem(WBaddress,WBPTR);
				delete [] WBPTR;
		}

			cache[block_offset+replace][0] = 1;
			cache[block_offset+replace][1] = 0;
			cache[block_offset+replace][3] = address_tag; //!!! to get address!
			counter++;
			cache[block_offset+replace][2] = counter;

	for (int j = 0; j < 4; j++)
	{
			int* addressPTR = mem->getfromMem(off_set);
				cache[block_offset+replace][4 + j] = addressPTR[j];
	}
	return cache[block_offset+replace][4 + data_allocate]; 
}

void L1cache::writetoCache(const int address,const int indata){
	//==================================================
        //===                   TODO                     ===

        //==================================================
	cout<<"printing cache content.....  (load address : "<<address<<')'<<endl;
	for(int i=0; i<L1size/2; i++)
	{
		cout<<i<<" : ";
		for(int j=0; j<8; j++)
			cout<<"["<<cache[i][j]<<"]";
		cout<<"  \t"<<i+4<<" : ";
		for(int j=0; j<8; j++)
			cout<<"["<<cache[i+4][j]<<"]";
		cout<<endl;
	}
	
	int W_finish = 0;
	int data_allocate = address%4;
	int off_set = address / 4;			// 給的address要先拿掉後面兩個bit
	int set_total=L1size/way_number; //把size/way可以得到到底有幾個set
	int index = off_set %set_total;   //要從address那拿到index 就把它mod settotal
	int block_offset = index*way_number; //index乘一個set有多少就是第幾個index的offset
	int address_tag = off_set /set_total;
	int counter=0;
	int replace=0;
	
	for(int i = block_offset; i<block_offset + way_number; i++)
	{
		if((cache[i][0] == 0)|| (cache[i][3] != address_tag))
			continue;
		else 
		{
			L1writehit++;
			cache[i][2] = counter;
			counter++;
			cache[i][1] = 1;
			cache[i][4 + data_allocate] = indata;
			W_finish = 1;
			break;
		}	
	}

	for(int i = block_offset ; i < block_offset + way_number; i++)
		{			
			if(cache[i][0] == 1)
				continue;
			else
			{
				cache[i][0] = 1;
				cache[i][1] = 1;
				cache[i][2] = counter;
				counter++;
				cache[i][3] = address_tag;
					for (int j = 0; j < 4; j++)
				{int* addressPTR = mem->getfromMem(off_set);
					cache[i][4 + j] = addressPTR[j];}

				cache[i][4 + data_allocate] = indata;
				W_finish = 1;
				
				break;
			}
		}

	
	
	if(W_finish == 0)
	{
	L1writemiss++;


	for (int i = 0; i < way_number;i++)
	{
		int temp=cache[block_offset][2];
		if (cache[block_offset+i][2]>=temp)
			continue;
		else
		{
			temp=cache[block_offset+i][2];
			replace=i;
		}

	} //then we can get which block need to be replace;


		if (cache[block_offset+replace][1] == 1)
		{
			int WBaddress = cache[block_offset+replace][3]*set_total+index;
			int* WBPTR = new int[4];
				for(int j=0; j<4; j++)
					WBPTR[j] = cache[block_offset+replace][4+j];
				  	mem->writetoMem(WBaddress, WBPTR);
				delete [] WBPTR;
			}
					
		cache[block_offset+replace][0] = 1;
		cache[block_offset+replace][2] = counter;
		counter++;
		cache[block_offset+replace][3] = address_tag;

		int* addressPTR = mem->getfromMem(off_set);

		for (int j = 0; j < 4; j++)
		{
			cache[block_offset+replace][4 + j] = addressPTR[j];
		}

		cache[block_offset+replace][4 + data_allocate] = indata;
		cache[block_offset+replace][1] = 1;
		W_finish = 1;
	
	for(int i = 0;i<L1size;i++)
	{
		if(cache[i][1] == 1)
		{
			int* dataPTR = new int[4];
				for(int j=0; j<4; j++)
					{dataPTR[j] = cache[i][4+j];
					mem->writetoMem(off_set, dataPTR);}
			delete [] dataPTR;
		}
	}
}
}


int L1cache::getReadHit(void){
	int temp = L1readhit;
	return temp;
}
int L1cache::getReadMiss(void){
	int temp = L1readmiss;
	return temp;
}
int L1cache::getWriteHit(void){
	int temp = L1writehit;
	return temp;
}
int L1cache::getWriteMiss(void){
	int temp = L1writemiss;
	return temp;
}
int L1cache::getHit(void){
	int temp = L1readhit + L1writehit;
	return temp;
}
int L1cache::getMiss(void){
	int temp = L1readmiss + L1writemiss;
	return temp;
}
