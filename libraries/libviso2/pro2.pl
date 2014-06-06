#!/usr/bin/perl
open(IN, "< projections");
while(<IN>)
{
    ($filename) = ($_ =~ /image\-(\d+)\.png/);
    print $filename, "\n";
    $result = sprintf("%04d", $filename);
    open(OUT, "> data/$result.txt");    
    print OUT "CONTOUR\n";
    for(1..3)
    {
	(@line) = split('\s+', <IN>);       
	print OUT join(" ", @line), "\n";	
    }
    <IN>;
    close(OUT);   
}
