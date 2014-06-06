#!/usr/bin/perl
open(IN, "< nnnn");
while(<IN>)
{
    if(/Current\spose/)
    {
	for(1..3)
	{
	    (@line) = split('\s+', <IN>);
	    print @line[4], " ";
	}
	<IN>;
	print "\n";
    }		
}
