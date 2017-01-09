
set val(chan) Channel/WirelessChannel ;# channel type
set val(prop) Propagation/TwoRayGround ;# radio-propagation model: TwoRayGround/FreeSpace
set val(netif) Phy/WirelessPhy ;# network interface type
set val(mac) Mac ;# MAC type
set val(ifq) Queue/DropTail/PriQueue ;# interface queue type
set val(ll) LL ;# link layer type
set val(ant) Antenna/OmniAntenna ;# antenna model
set val(ifqlen) 1000 ;# max packet in ifq
set val(nn) [lindex $argv 1] ;# number of mobilenodes
set val(rp) DumbAgent ;# routing protocol
set val(x) 30 ;# X dimension of topography
set val(y) 30 ;# Y dimension of topography 
set val(stop) 60 ;# time of simulation end

set ns [new Simulator]

set nf [open out.nam w]
$ns namtrace-all $nf


set n0 [$ns node]
set n1 [$ns node]

set topo [new Topography]

proc finish {} {
        global ns nf
        $ns flush-trace
        close $nf
        exec nam out.nam &
        exit 0
}



$ns at 5.0 "finish"

$ns run
