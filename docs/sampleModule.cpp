void FullAdder::dolt(void{
  sc_int<16> tmp_A,tmp_B;
  sc_int<17> tmp_R;

  tmp_A = (sc_int<16>) A.read();
  tmp_B = (sc_int<16>) B.read();

  tmp_R = tmp_A + tmp_B;

  result.write((sc_uint<16>) tmp_R.range(15,0));
}
