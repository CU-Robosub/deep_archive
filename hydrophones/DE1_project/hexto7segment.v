
module hexto7segment(
    input  [7:0]x,
    output reg [6:0]z1,z2
    );
always @* begin
case (x[3:0])
4'b0000 :      	//Hexadecimal 0
z1 = ~7'b1111110;
4'b0001 :    		//Hexadecimal 1
z1 = ~7'b0110000  ;
4'b0010 :  		// Hexadecimal 2
z1 = ~7'b1101101 ; 
4'b0011 : 		// Hexadecimal 3
z1 = ~7'b1111001 ;
4'b0100 :		// Hexadecimal 4
z1 = ~7'b0110011 ;
4'b0101 :		// Hexadecimal 5
z1 = ~7'b1011011 ;  
4'b0110 :		// Hexadecimal 6
z1 = ~7'b1011111 ;
4'b0111 :		// Hexadecimal 7
z1 = ~7'b1110000;
4'b1000 :     		 //Hexadecimal 8
z1 = ~7'b1111111;
4'b1001 :    		//Hexadecimal 9
z1 = ~7'b1111011 ;
4'b1010 :  		// Hexadecimal A
z1 = ~7'b1110111 ; 
4'b1011 : 		// Hexadecimal B
z1 = ~7'b0011111;
4'b1100 :		// Hexadecimal C
z1 = ~7'b1001110 ;
4'b1101 :		// Hexadecimal D
z1 = ~7'b0111101 ;
4'b1110 :		// Hexadecimal E
z1 = ~7'b1001111 ;
4'b1111 :		// Hexadecimal F
z1 = ~7'b1000111 ;
endcase

case (x[7:4])
4'b0000 :      	//Hexadecimal 0
z2 = ~7'b1111110;
4'b0001 :    		//Hexadecimal 1
z2 = ~7'b0110000  ;
4'b0010 :  		// Hexadecimal 2
z2 = ~7'b1101101 ; 
4'b0011 : 		// Hexadecimal 3
z2 = ~7'b1111001 ;
4'b0100 :		// Hexadecimal 4
z2 = ~7'b0110011 ;
4'b0101 :		// Hexadecimal 5
z2 = ~7'b1011011 ;  
4'b0110 :		// Hexadecimal 6
z2 = ~7'b1011111 ;
4'b0111 :		// Hexadecimal 7
z2 = ~7'b1110000;
4'b1000 :     		 //Hexadecimal 8
z2 = ~7'b1111111;
4'b1001 :    		//Hexadecimal 9
z2 = ~7'b1111011 ;
4'b1010 :  		// Hexadecimal A
z2 = ~7'b1110111 ; 
4'b1011 : 		// Hexadecimal B
z2 = ~7'b0011111;
4'b1100 :		// Hexadecimal C
z2 = ~7'b1001110 ;
4'b1101 :		// Hexadecimal D
z2 = ~7'b0111101 ;
4'b1110 :		// Hexadecimal E
z2 = ~7'b1001111 ;
4'b1111 :		// Hexadecimal F
z2 = ~7'b1000111 ;
endcase
end
 
endmodule