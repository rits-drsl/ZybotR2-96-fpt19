/*
 MIT License

 Copyright (c) 2019 Yuya Kudo

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 */

//-----------------------------------------------------------------------------
// bus for write port
interface w_busif
  #(parameter
    DATA_WIDTH   = 32,
    RAM_DEPTH    = 256,
    localparam
    LB_RAM_DEPTH = $clog2(RAM_DEPTH));

   logic [DATA_WIDTH-1:0]    data;
   logic [LB_RAM_DEPTH-1:0]  addr;
   logic                     valid;
   logic                     ready;

   modport master(output data,
                  output addr,
                  output valid,
                  input  ready);

   modport slave(input  data,
                 input  addr,
                 input  valid,
                 output ready);

endinterface
