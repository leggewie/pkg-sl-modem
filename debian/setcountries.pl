#!/usr/bin/perl

chomp( $list=`modem/slmodemd --countrylist 2>&1`);
$list =~s/\w+: //g;
$list =~s/\n/, /g;
open(t, "+<".$ARGV[0]);
@cont=<t>;
seek(t,0,0);
for(@cont) {
   $_=~s/_list_/$list/;
   print t $_;
}
close(t);
