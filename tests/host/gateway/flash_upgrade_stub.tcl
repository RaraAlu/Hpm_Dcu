# Interpret the generated transaction against a file, not a target.
lassign $argv transaction flash_file elf payload_file base erase_start journal_address fault

proc load_binary {path} {
    set fd [open $path rb]
    set data [read $fd]
    close $fd
    return $data
}

proc save_binary {path data} {
    set fd [open $path wb]
    puts -nonewline $fd $data
    close $fd
}

set contents [load_binary $flash_file]
set payload [load_binary $payload_file]

proc view {addr size} {
    global contents base
    set start [expr {$addr - $base}]
    if {$start < 0 || $start + $size > [string length $contents]} {error "read outside Flash"}
    return [string range $contents $start [expr {$start + $size - 1}]]
}

proc replace {addr data} {
    global contents base flash_file
    set start [expr {$addr - $base}]
    set contents [string replace $contents $start [expr {$start + [string length $data] - 1}] $data]
    save_binary $flash_file $contents
}

proc init {} {}
proc halt {} {}
proc echo {message} {puts $message}
proc shutdown {} {}
proc reset {mode} {
    global journal_address fault
    if {$mode eq "init" && $fault eq "reset_changed_journal"} {
        replace $journal_address [binary format H* ff]
    }
}

proc flash {command args} {
    global contents elf payload erase_start journal_address fault
    switch -exact -- $command {
        list {return [list [dict create size [string length $contents]]]}
        probe - info {return}
        write_image {
            if {$args ne [list erase $elf]} {error "unexpected image command"}
            puts "APP_WRITE"
            replace $erase_start $payload
            if {$fault eq "erase_spill"} {
                replace $journal_address [binary format H* ff]
            }
        }
        default {error "unexpected flash command"}
    }
}

proc dump_image {path addr size} {
    global journal_address fault
    if {$addr == $journal_address && $fault eq "backup_failed"} {error "backup failed"}
    if {$addr == $journal_address && $fault eq "truncated_backup"} {incr size -1}
    save_binary $path [view $addr $size]
}

proc verify_image {path args} {
    global elf payload erase_start
    if {$path eq $elf && [llength $args] == 0} {
        set addr $erase_start
        set expected $payload
    } elseif {[llength $args] == 2 && [lindex $args 1] eq "bin"} {
        set addr [lindex $args 0]
        set expected [load_binary $path]
    } else {
        error "unexpected verify command"
    }
    if {[view $addr [string length $expected]] ne $expected} {error "verification failed: $path"}
}

source $transaction
