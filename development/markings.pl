#!/usr/bin/perl

use strict;
use warnings;


#  CBA  UTS
#  FED  XWV
#  IHG  &ZY
#  LKJ  321
#  ONM  654
#  RQP  087


foreach my $letter ('0'..'9', 'A'..'Z', '&') {
  my $code = ord($letter);
  printf "%s  ->  %06b  (%07b)\n", $letter, $code & 077, $code;
}
