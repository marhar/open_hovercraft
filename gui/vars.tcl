puts [glob /dev/cu.*SLAB*]
puts [glob /dev/cu.*HC-0*]

#-----------------------------------------------------------------------------
set device [glob /dev/cu.*SLAB*]
set port [open $device r+]
fconfigure $port -mode 9600,n,8,1 -blocking 0 -translation auto -buffering none
fileevent $port readable [list read_port $port]

set remote_line ""
proc read_port {port} {
  set data [read $port 1]
  if {[string length $data] > 0} { set ::remote_line $::remote_line$data }
  if {$data == "\n"} {
    process_line $::remote_line
    set ::line ""
  }
}

#-----------------------------------------------------------------------------
proc process_line {line} {
  puts $line
  while {[regexp -indices {([A-Za-z0-9_]+:[0-9.-]+)} $line ix]} {
    set first [lindex $ix 0]
    set last [lindex $ix 1]
    set head [string range $line $first $last]
    set eqix [string first : $head]
    set key [string range $head 0 $eqix-1]
    set val [string range $head $eqix+1 end]
    process_kv $key $val
    set line [string range $line $last end]
  }
}

#-----------------------------------------------------------------------------
proc process_kv {key val} {
  if {[string compare $key pcoef]} {.pstuff.coef configure -text pcoef:$val;puts  $val}
  if {[string compare $key pcorr]} {.pstuff.corr configure -text pcorr:$val}
  if {[string compare $key icoef]} {.istuff.coef configure -text $val}
  if {[string compare $key icorr]} {.istuff.corr configure -text $val}
  if {[string compare $key dcoef]} {.dstuff.coef configure -text $val}
  if {[string compare $key dcorr]} {.dstuff.corr configure -text $val}
  update idletasks
}

pack [label .device -text $device]
  #$test.b insert end [read $port]
  #$test.b see insert

pack [frame .pstuff]
pack [frame .istuff]
pack [frame .dstuff]
pack [label .pstuff.coef -text ..] -side left
pack [label .pstuff.corr -text ..] -side left
pack [label .istuff.coef -text ..] -side left
pack [label .istuff.corr -text ..] -side left
pack [label .dstuff.coef -text ..] -side left
pack [label .dstuff.corr -text ..] -side left

