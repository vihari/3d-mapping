#!/usr/bin/perl
$file = 1;
open(IN, "< projData");
while(<IN>)
{
    if($_ =~ /CONTOUR/)
    {
	$result = sprintf("%04d", $file);
	close(OUT) if($file > 1);
	open(OUT, "> data/$result.txt");
	$file++;
    }
    print OUT $_ if(OUT);
}
