function bitfield(value, offset, len)
    return bit32.band(bit32.rshift(value, offset), bit32.lshift(1, len) - 1)
end


local sll_pkttype = Field.new("sll.pkttype")  -- Linux Cooked Capture Type
local can_id_field = Field.new("can.id")
local can_extended = Field.new("can.flags.xtd")


local canmore_protocol = Proto("canmore",  "CANmore Protocol")
local canmore_f = {
    clientid = ProtoField.uint8("canmore.clientid", "Client ID", base.DEC),
    type = ProtoField.uint8("canmore.type", "Type", base.DEC, { [1] = "Utility", [0] = "Message" }),
    dir = ProtoField.uint8("canmore.direction", "Direction", base.DEC, { [1] = "agent->client", [0] = "client->agent"}),
    chan = ProtoField.uint8("canmore.channel", "Channel", base.DEC),
    seqnum = ProtoField.uint8("canmore.seq_num", "Msg Seq Num", base.DEC),
    extra = ProtoField.uint32("canmore.extra", "Extra ID", base.HEX)
}
canmore_protocol.fields = canmore_f
function canmore_protocol.dissector(buffer, pinfo, tree)
    if sll_pkttype() ~= nil and (sll_pkttype().value ~= 1) then
        -- Only decode type 1 packets (Linux Cooked Capture Broadcast Packets)
        return
    end

    local is_extended = can_extended().value
    local can_id = bit32.band(can_id_field().value, 0x1FFFFFFF)
    local clientid_val
    local type_val
    local dir_val
    local noc_val
    local extra_val

    pinfo.cols.protocol:set("CANmore")

    if (is_extended) then
        clientid_val = bitfield(can_id, 24, 5)
        type_val = bitfield(can_id, 23, 1)
        dir_val = bitfield(can_id, 22, 1)
        noc_val = bitfield(can_id, 18, 4)
        extra_val = bitfield(can_id, 0, 18)
    else
        clientid_val = bitfield(can_id, 6, 5)
        type_val = bitfield(can_id, 5, 1)
        dir_val = bitfield(can_id, 4, 1)
        noc_val = bitfield(can_id, 0, 4)
    end

    local subtree_title = "CANmore: Client " .. tostring(clientid_val) .. ", "
    if (dir_val == 1) then
        subtree_title = subtree_title .. "Agent->Client, "
    else
        subtree_title = subtree_title .. "Client->Agent, "
    end
    if (type_val == 1) then
        subtree_title = subtree_title .. "Utility Ch " .. tostring(noc_val)
    else
        subtree_title = subtree_title ..  "Msg Seq " .. tostring(noc_val)
    end


    local subtree = tree:add(canmore_protocol, buffer(0, 0), subtree_title)
    subtree:add(canmore_f.clientid, clientid_val)
    subtree:add(canmore_f.type, type_val)
    subtree:add(canmore_f.dir, dir_val)
    if (type_val == 1) then
        subtree:add(canmore_f.chan, noc_val)
    else
        subtree:add(canmore_f.seqnum, noc_val)
    end
    if (is_extended) then
        subtree:add(canmore_f.extra, extra_val)
    end

    pinfo.private["canmore_dir"] = dir_val

    local info_msg = ""
    if (type_val == 1) then
        info_msg = info_msg .. "Utility"
    else
        info_msg = info_msg .. "Message"
    end
    info_msg = info_msg .. " frame - "
    if (type_val == 1) then
        info_msg = info_msg .. "Channel " .. tostring(noc_val)
    else
        info_msg = info_msg .. "Seq " .. tostring(noc_val)
    end

    pinfo.cols.info:set(info_msg)


    if (dir_val == 1) then
        pinfo.cols.src:set("Agent")
        pinfo.cols.dst:set("Client " .. tostring(clientid_val))
    else
        pinfo.cols.src:set("Client " .. tostring(clientid_val))
        pinfo.cols.dst:set("Agent")
    end

    if (type_val == 1) and (noc_val == 15) then
        Dissector.get("canmore.heartbeat"):call(buffer, pinfo, tree)
    elseif (type_val == 1) and (noc_val == 14) then
        Dissector.get("canmore.reg_mapped"):call(buffer, pinfo, tree)
    elseif (type_val == 1) and (noc_val == 13) then
        Dissector.get("remotetty"):call(buffer, pinfo, tree)
    else
        Dissector.get("data"):call(buffer, pinfo, tree)
    end

end

local can_subdissector = DissectorTable.get("can.subdissector")
can_subdissector:add_for_decode_as(canmore_protocol)


local canmore_heartbeat = Proto("canmore.heartbeat", "CANmore Heartbeat")
local heartbeat_mode_table = {
    [0] = "No Control Interface Present",
    [1] = "Normal Mode",
    [2] = "Bootloader Mode",
    [3] = "Linux Mode",
    [5] = "Boot Delay"
}
local heartbeat_f = {
    cnt = ProtoField.uint8("canmore.heartbeat.cnt", "Heartbeat Count", base.DEC),
    err = ProtoField.uint8("canmore.heartbeat.err", "Error State", base.DEC, { [0] = "Okay", [1] = "Fault" }),
    mode = ProtoField.uint8("canmore.heartbeat.mode", "Client Mode", base.DEC, heartbeat_mode_table),
    term_valid = ProtoField.uint8("canmore.heartbeat.term_valid", "Term State Valid", base.DEC, { [0] = "Not Valid", [1] = "Valid"}),
    term_enabled = ProtoField.uint8("canmore.heartbeat.term_enabled", "Term Resistor Enabled", base.DEC, { [0] = "Enabled", [1] = "Disabled"})
}
canmore_heartbeat.fields = heartbeat_f
function canmore_heartbeat.dissector(buffer, pinfo, tree)
    if (buffer:len() ~= 1) then
        return
    end

    local heartbeat_data = buffer(0, 1):range(0, 1):uint()
    local cnt_val = bitfield(heartbeat_data, 0, 2)
    local err_val = bitfield(heartbeat_data, 2, 1)
    local mode_val = bitfield(heartbeat_data, 3, 3)
    local term_valid_val = bitfield(heartbeat_data, 6, 1)
    local term_enabled_val = bitfield(heartbeat_data, 7, 1)

    local tree_title = "Heartbeat: "
    if heartbeat_mode_table[mode_val] ~= nil then
        tree_title = tree_title .. heartbeat_mode_table[mode_val]
    else
        tree_title = tree_title .. "Unknown Mode (" .. tostring(mode_val) .. ")"
    end
    tree_title = tree_title .. ", Count: " .. tostring(cnt_val)
    if err_val ~= 0 then
        tree_title = tree_title .. ", Fault Present"
    end

    local subtree = tree:add(canmore_heartbeat, buffer, "CANmore " .. tree_title)
    subtree:add(heartbeat_f.cnt, cnt_val)
    subtree:add(heartbeat_f.err, err_val)
    subtree:add(heartbeat_f.mode, mode_val)
    subtree:add(heartbeat_f.term_valid, term_valid_val)
    if (bitfield(heartbeat_data, 6, 1) == 1) then
        subtree:add(heartbeat_f.term_enabled, term_enabled_val)
    end

    pinfo.cols.info:set(tree_title)
end




local remotetty_protocol = Proto("remotetty",  "CANmore Remote TTY")
subch_table = { [0] = "Control", [1] = "STDIN", [2] = "STDOUT", [3] = "STDERR"}
cmd_table = {[0] = "Disconnect", [1] = "Ack", [2] = "Window Size"}
local remotetty_f = {
    subch = ProtoField.uint8("remotetty.subch", "Subchannel", base.DEC, subch_table),
    cmd = ProtoField.uint8("remotetty.cmd", "Command", base.DEC, cmd_table),
    seq = ProtoField.uint8("remotetty.seq", "Sequence", base.DEC),
    data = ProtoField.string("remotetty.data", "Data", base.UNICODE),
    cmdrawdata = ProtoField.bytes("remotetty.cmddata", "Command Data")
}
remotetty_protocol.fields = remotetty_f
function remotetty_protocol.dissector(buffer, pinfo, tree)
    local can_id = bit32.band(can_id_field().value, 0x1FFFFFFF)

    local subch_val = bitfield(can_id, 16, 2)
    local seqcmd_val = bitfield(can_id, 0, 16)

    pinfo.cols.protocol:set("Remote TTY")

    local subtree_title = subch_table[subch_val] .. " Channel"
    if (subch_val == 0) then
        subtree_title = subtree_title .. ", "
        if cmd_table[seqcmd_val] ~= nil then
            subtree_title = subtree_title .. "Command: " .. cmd_table[seqcmd_val]
        else
            subtree_title = subtree_title .. "Unknown Command (" .. tostring(seqcmd_val) .. ")"
        end
    else
        subtree_title = subtree_title .. ", Seq Num: " .. tostring(seqcmd_val)
    end


    local subtree = tree:add(canmore_protocol, buffer(0, -1), "Remote TTY: " .. subtree_title)
    subtree:add(remotetty_f.subch, subch_val)
    if subch_val == 0 then
        subtree:add(remotetty_f.cmd, seqcmd_val)
        subtree:add(remotetty_f.cmdrawdata, buffer(0, -1))
    else
        subtree:add(remotetty_f.seq, seqcmd_val)
        subtree:add(remotetty_f.data, buffer(0, -1))
    end

    pinfo.cols.info:set(subtree_title)

end



local reg_mapped_protocol = Proto("canmore.reg_mapped", "CANmore Reg Mapped")
local reg_mapped_f = {
    access = ProtoField.uint8("canmore.reg_mapped.access", "Access Type", base.DEC, { [0] = "Read", [1] = "Write"}),
    bulk_req = ProtoField.bool("canmore.reg_mapped.bulk_req", "Bulk Request"),
    bulk_end = ProtoField.bool("canmore.reg_mapped.bulk_end", "Bulk End"),
    multiword = ProtoField.bool("canmore.reg_mapped.multiword", "Multiword"),
    mode = ProtoField.uint8("canmore.reg_mapped.mode", "Client Mode", base.DEC, heartbeat_mode_table),
    count = ProtoField.uint8("canmore.reg_mapped.count", "Sequence Count", base.DEC),
    addr_page = ProtoField.uint8("canmore.reg_mapped.addr_page", "Reg Page", base.DEC),
    addr_offset = ProtoField.uint8("canmore.reg_mapped.addr_offset", "Reg Offset", base.DEC),
    write_data = ProtoField.uint32("canmore.reg_mapped.wdata", "Write Data", base.HEX)
}
reg_mapped_protocol.fields = reg_mapped_f
function reg_mapped_protocol.dissector(buffer, pinfo, tree)
    if (buffer:len() == 0) then
        return
    end

    pinfo.cols.protocol:set("Reg Mapped")

    if (pinfo.private["canmore_dir"] == "1") then
        local flag_data = buffer(0, 1):range(0, 1):uint()
        local access_val = bitfield(flag_data, 0, 1)
        local bulk_req_val = bitfield(flag_data, 1, 1)
        local bulk_end_val = bitfield(flag_data, 2, 1)
        local multiword_val = bitfield(flag_data, 3, 1)
        local mode_val = bitfield(flag_data, 5, 3)

        local count_val = buffer:range(1, 1):uint()
        local addr_page_val = buffer:range(2, 1):uint()
        local addr_offset_val = buffer:range(3, 1):uint()

        if (access_val == 1) and (buffer:len() ~= 8) then
            return
        elseif (access_val == 0) and (buffer:len() ~= 4) then
            return
        end

        -- local heartbeat_data = buffer(0, 1):range(0, 1):uint()

        local tree_title = ""
        if access_val == 1 then
            tree_title = tree_title .. "Write"
        else
            tree_title = tree_title .. "Read"
        end
        tree_title = tree_title .. " Page " .. tostring(addr_page_val) .. ", Offset: " .. tostring(addr_offset_val) .. ", "
        if heartbeat_mode_table[mode_val] ~= nil then
            tree_title = tree_title .. heartbeat_mode_table[mode_val]
        else
            tree_title = tree_title .. "Unknown Mode (" .. tostring(mode_val) .. ")"
        end
        if bulk_req_val == 1 then
            tree_title = tree_title .. ", Seq Count: " .. tostring(count_val)
        end

        local subtree = tree:add(reg_mapped_protocol, buffer, "CANmore Reg Mapped: " .. tree_title)
        subtree:add(reg_mapped_f.access, access_val)
        subtree:add(reg_mapped_f.bulk_req, bulk_req_val == 1)
        subtree:add(reg_mapped_f.bulk_end, bulk_end_val == 1)
        subtree:add(reg_mapped_f.multiword, multiword_val == 1)
        subtree:add(reg_mapped_f.mode, mode_val)
        subtree:add(reg_mapped_f.addr_page, addr_page_val)
        subtree:add(reg_mapped_f.addr_offset, addr_offset_val)
        if bulk_req_val == 1 then
            subtree:add(reg_mapped_f.count, count_val)
        end

        pinfo.cols.info:set(tree_title)
    end
end
