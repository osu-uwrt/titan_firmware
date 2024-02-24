-- Wireshark Dissector for CANmore
-- Allows decoding of CANmore bus captures using Wireshark
--
-- This script can be installed by symlinking this file to ~/.local/lib/wireshark/plugins on Linux or copying this to
-- the %APPDATA%\Wireshark\plugins folder on Windows.
--
-- These captures can be directly captured with Wireshark, or captured using `candump -l any,0:0,#FFFFFFFF` on a remote
-- machine (such as the orin) and later opened in Wireshark. These logs will start with the filename "candump-".
-- You must enable the dissector by going to Decode As -> CAN Next Level Dissector -> Set Current to CANMORE. If not,
-- all traffic will appear as raw CAN frames.

local plguin_info = {
    version = "1.0.0",
    author = "Robert Pafford",
    repository = "https://github.com/osu-uwrt/titan_firmware",
    description = "Decodes CANmore frames used by Titan Firmware"
}

set_plugin_info(plguin_info)

function bitfield(value, offset, len)
    return bit32.band(bit32.rshift(value, offset), bit32.lshift(1, len) - 1)
end

local bool_table = {
    [0] = "False",
    [1] = "True"
}

-- ========================================
-- CANmore Core Decoder
-- ========================================

local canmore_protocol = Proto("canmore",  "CANmore Protocol")
local canmore_f = {
    raw_id = ProtoField.uint16("canmore.raw_id", "Standard ID", base.HEX, nil, nil, "Raw CAN ID for this frame"),
    clientid = ProtoField.uint16("canmore.clientid", "Client ID", base.DEC, nil, 0x7C0, "The source/destination client for this frame"),
    type = ProtoField.uint16("canmore.type", "Type", base.DEC, { [1] = "Utility", [0] = "Message" }, 0x020, "The CANmore frame type, either message or utility"),
    dir = ProtoField.uint16("canmore.direction", "Direction", base.DEC, { [1] = "agent->client", [0] = "client->agent" }, 0x010),
    chan = ProtoField.uint16("canmore.channel", "Channel", base.DEC, nil, 0x00F, "The utility channel for this frame"),
    seqnum = ProtoField.uint16("canmore.seq_num", "Msg Seq Num", base.DEC, nil, 0x00F, "The message sequence number for this frame"),

    raw_id_ext = ProtoField.uint16("canmore.raw_id", "Extended ID", base.HEX, nil, nil, "Raw CAN ID for this frame"),
    clientid_ext = ProtoField.uint32("canmore.clientid", "Client ID", base.DEC, nil, 0x1F000000, "The source/destination client for this frame"),
    type_ext = ProtoField.uint32("canmore.type", "Type", base.DEC, { [1] = "Utility", [0] = "Message" }, 0x00800000, "The CANmore frame type, either message or utility"),
    dir_ext = ProtoField.uint32("canmore.direction", "Direction", base.DEC, { [1] = "agent->client", [0] = "client->agent"}, 0x00400000),
    chan_ext = ProtoField.uint32("canmore.channel", "Channel", base.DEC, nil, 0x003C0000, "The utility channel for this frame"),
    seqnum_ext = ProtoField.uint32("canmore.seq_num", "Msg Seq Num", base.DEC, nil, 0x003C0000, "The message sequence number for this frame"),
    extra = ProtoField.uint32("canmore.extra", "Extra Data", base.HEX, nil, 0x0003FFFF)
}
canmore_protocol.fields = canmore_f

-- Fields extracted from the can protocol since we aren't directly passed this information
-- These should only be used by the CANmore core decoder
local sll_pkttype = Field.new("sll.pkttype")  -- Linux Cooked Capture Type
local can_id_field = Field.new("can.id")
local can_extended = Field.new("can.flags.xtd")

function canmore_protocol.dissector(buffer, pinfo, tree)
    if sll_pkttype() ~= nil and (sll_pkttype().value ~= 1) then
        -- Only decode type 1 packets (Linux Cooked Capture Broadcast Packets)
        -- If not, we'll get duplicate packets because wireshark sees both the transmit and the broadcast back out
        return 0
    end

    local is_extended = can_extended().value
    local can_id = bit32.band(can_id_field().value, 0x1FFFFFFF)
    local can_id_tvb = can_id_field().range
    local clientid_val
    local type_val
    local dir_val
    local noc_val
    local extra_val

    pinfo.cols.protocol:set("CANmore")

    if is_extended then
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
    if dir_val == 1 then
        subtree_title = subtree_title .. "Agent->Client, "
    else
        subtree_title = subtree_title .. "Client->Agent, "
    end
    if type_val == 1 then
        subtree_title = subtree_title .. "Utility Ch " .. tostring(noc_val)
    else
        subtree_title = subtree_title ..  "Msg Seq " .. tostring(noc_val)
    end

    -- Decode the CAN id
    local subtree = tree:add(canmore_protocol, can_id_tvb, subtree_title)
    if is_extended then
        local can_id_tree = subtree:add(canmore_f.raw_id_ext, can_id_tvb, can_id)
        can_id_tree:add(canmore_f.clientid_ext, can_id_tvb, can_id)
        can_id_tree:add(canmore_f.type_ext, can_id_tvb, can_id)
        can_id_tree:add(canmore_f.dir_ext, can_id_tvb, can_id)
        if (type_val == 1) then
            can_id_tree:add(canmore_f.chan_ext, can_id_tvb, can_id)
        else
            can_id_tree:add(canmore_f.seqnum_ext, can_id_tvb, can_id)
        end
        can_id_tree:add(canmore_f.extra, can_id_tvb, can_id)
    else
        local can_id_tree = subtree:add(canmore_f.raw_id, can_id_tvb, can_id)
        can_id_tree:add(canmore_f.clientid, can_id_tvb, can_id)
        can_id_tree:add(canmore_f.type, can_id_tvb, can_id)
        can_id_tree:add(canmore_f.dir, can_id_tvb, can_id)
        if type_val == 1 then
            can_id_tree:add(canmore_f.chan, can_id_tvb, can_id)
        else
            can_id_tree:add(canmore_f.seqnum, can_id_tvb, can_id)
        end
    end

    pinfo.private["canmore_client_id"] = clientid_val
    pinfo.private["canmore_dir"] = dir_val
    pinfo.private["canmore_extra"] = extra_val
    pinfo.private["canmore_noc"] = noc_val

    local info_msg = ""
    if type_val == 1 then
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


    if dir_val == 1 then
        pinfo.cols.src:set("Agent")
        pinfo.cols.dst:set("Client " .. tostring(clientid_val))
    else
        pinfo.cols.src:set("Client " .. tostring(clientid_val))
        pinfo.cols.dst:set("Agent")
    end

    if type_val == 0 then
        Dissector.get("canmore.msg"):call(buffer, pinfo, tree)
    elseif (type_val == 1) and (noc_val == 15) then
        Dissector.get("canmore.heartbeat"):call(buffer, pinfo, tree)
    elseif (type_val == 1) and (noc_val == 14) then
        Dissector.get("canmore.reg_mapped"):call(buffer, pinfo, tree)
    elseif (type_val == 1) and (noc_val == 13) then
        Dissector.get("canmore.remotetty"):call(buffer, pinfo, tree)
    else
        Dissector.get("data"):call(buffer, pinfo, tree)
    end

end

local can_subdissector = DissectorTable.get("can.subdissector")
can_subdissector:add_for_decode_as(canmore_protocol)


-- ========================================
-- CANmore Heartbeat Decoder
-- ========================================

local canmore_heartbeat = Proto("canmore.heartbeat", "CANmore Heartbeat")
local heartbeat_mode_table = {
    [0] = "No Control Interface Present",
    [1] = "Normal Mode",
    [2] = "Bootloader Mode",
    [3] = "Linux Mode",
    [5] = "Boot Delay"
}
local heartbeat_f = {
    raw = ProtoField.uint8("canmore.heartbeat.raw", "Heartbeat Status", base.HEX, nil, nil, "Raw status field of the heartbeat frame"),
    cnt = ProtoField.uint8("canmore.heartbeat.cnt", "Heartbeat Count", base.DEC, nil, 0x03, "Incrementing 2-bit counter"),
    err = ProtoField.uint8("canmore.heartbeat.err", "Error State", base.DEC, { [0] = "Okay", [1] = "Fault" }, 0x04, "Error Present Flag"),
    mode = ProtoField.uint8("canmore.heartbeat.mode", "Client Mode", base.DEC, heartbeat_mode_table, 0x38, "The reg mapped client mode implemented by the client"),
    term_valid = ProtoField.uint8("canmore.heartbeat.term_valid", "Term State Valid", base.DEC, { [0] = "Not Valid", [1] = "Valid"}, 0x40, "CAN Bus termination resistor state is valid flag"),
    term_enabled = ProtoField.uint8("canmore.heartbeat.term_enabled", "Term Resistor Enabled", base.DEC, { [0] = "Enabled", [1] = "Disabled"}, 0x80, "CAN Bus termination resistor state flag")
}
local heartbeat_e = {
    malformed_pkt = ProtoExpert.new("canmore.heartbeat.malformed", "Invalid Heartbeat Length", expert.group.MALFORMED, expert.severity.ERROR),
}
canmore_heartbeat.fields = heartbeat_f
canmore_heartbeat.experts = heartbeat_e
function canmore_heartbeat.dissector(buffer, pinfo, tree)
    if (buffer:len() ~= 1) then
        pinfo.cols.info:set("Heartbeat [MALFORMED]")
        local subtree = tree:add(canmore_heartbeat, buffer(), "CANmore Heartbeat [MALFORMED]")
        subtree:add_proto_expert_info(heartbeat_e.malformed_pkt)
        return 0
    end

    local heartbeat_tvb = buffer(0, 1)
    local heartbeat_data = heartbeat_tvb:uint()
    local cnt_val = bitfield(heartbeat_data, 0, 2)
    local err_val = bitfield(heartbeat_data, 2, 1)
    local mode_val = bitfield(heartbeat_data, 3, 3)

    local tree_title
    if heartbeat_mode_table[mode_val] ~= nil then
        tree_title = heartbeat_mode_table[mode_val]
    else
        tree_title = "Unknown Mode (" .. tostring(mode_val) .. ")"
    end
    tree_title = tree_title .. ", Count: " .. tostring(cnt_val)
    if err_val ~= 0 then
        tree_title = tree_title .. ", Fault Present"
    end

    local subtree = tree:add(canmore_heartbeat, buffer(), "CANmore Heartbeat: " .. tree_title)
    local heartbeat_byte = subtree:add(heartbeat_f.raw, heartbeat_tvb)
    heartbeat_byte:add(heartbeat_f.cnt, heartbeat_tvb)
    heartbeat_byte:add(heartbeat_f.err, heartbeat_tvb)
    heartbeat_byte:add(heartbeat_f.mode, heartbeat_tvb)
    heartbeat_byte:add(heartbeat_f.term_valid, heartbeat_tvb)
    if (bitfield(heartbeat_data, 6, 1) == 1) then
        heartbeat_byte:add(heartbeat_f.term_enabled, heartbeat_tvb)
    end

    pinfo.cols.protocol:set("Heartbeat")
    pinfo.cols.info:set(tree_title)
    return buffer:len()
end


-- ========================================
-- CANmore Remote TTY Decoder
-- ========================================

local remotetty_protocol = Proto("canmore.remotetty",  "CANmore Remote TTY")
subch_table = { [0] = "Control", [1] = "STDIN", [2] = "STDOUT", [3] = "STDERR"}
cmd_table = {[0] = "Disconnect", [1] = "Ack", [2] = "Window Size"}
local remotetty_f = {
    subch = ProtoField.uint8("canmore.remotetty.subch", "Subchannel", base.DEC, subch_table),
    cmd = ProtoField.uint8("canmore.remotetty.cmd", "Command", base.DEC, cmd_table),
    seq = ProtoField.uint8("canmore.remotetty.seq", "Sequence", base.DEC),
    data = ProtoField.string("canmore.remotetty.data", "Data", base.UNICODE),
    cmdrawdata = ProtoField.bytes("canmore.remotetty.cmddata", "Command Data")
}
remotetty_protocol.fields = remotetty_f
function remotetty_protocol.dissector(buffer, pinfo, tree)
    local id_extra = tonumber(pinfo.private["canmore_extra"])
    local can_id_tvb = can_id_field().range

    local subch_val = bitfield(id_extra, 16, 2)
    local seqcmd_val = bitfield(id_extra, 0, 16)

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


    local subtree = tree:add(remotetty_protocol, buffer(), "Remote TTY: " .. subtree_title)
    subtree:add(remotetty_f.subch, can_id_tvb, subch_val)
    if subch_val == 0 then
        subtree:add(remotetty_f.cmd, can_id_tvb, seqcmd_val)
        subtree:add(remotetty_f.cmdrawdata, buffer(0, -1))
    else
        subtree:add(remotetty_f.seq, can_id_tvb, seqcmd_val)
        subtree:add(remotetty_f.data, buffer(0, -1))
    end

    pinfo.cols.info:set(subtree_title)
    return buffer:len()
end


-- ========================================
-- CANmore Reg Mapped Decoder
-- ========================================

local reg_mapped_protocol = Proto("canmore.reg_mapped", "CANmore Reg Mapped")
local reg_mapped_result_table = {
    [0] = "Successful",
    [1] = "Malformed Request",
    [2] = "Bulk Request Sequence Error",
    [3] = "Invalid Register Address",
    [4] = "Invalid Register Mode",
    [5] = "Invalid Data",
    [6] = "Invalid Mode"
}
local reg_mapped_f = {
    flags = ProtoField.uint8("canmore.reg_mapped.flags", "Flags", base.HEX),
    access = ProtoField.uint8("canmore.reg_mapped.flags.is_write", "Access Type", base.DEC, { [0] = "Read", [1] = "Write"}, 0x01),
    bulk_req = ProtoField.uint8("canmore.reg_mapped.flags.bulk_req", "Bulk Request", base.DEC, bool_table, 0x02),
    bulk_end = ProtoField.uint8("canmore.reg_mapped.flags.bulk_end", "Bulk End", base.DEC, bool_table, 0x04),
    multiword = ProtoField.uint8("canmore.reg_mapped.flags.multiword", "Multiword", base.DEC, bool_table, 0x08),
    mode = ProtoField.uint8("canmore.reg_mapped.flags.mode", "Client Mode", base.DEC, heartbeat_mode_table, 0xE0),
    count = ProtoField.uint8("canmore.reg_mapped.count", "Sequence Count", base.DEC),
    addr_page = ProtoField.uint8("canmore.reg_mapped.addr_page", "Reg Page", base.DEC),
    addr_offset = ProtoField.uint8("canmore.reg_mapped.addr_offset", "Reg Offset", base.DEC),
    write_data = ProtoField.uint32("canmore.reg_mapped.wdata", "Write Data", base.HEX),
    result = ProtoField.uint32("canmore.reg_mapped.result", "Result", base.DEC, reg_mapped_result_table),
    read_data = ProtoField.uint32("canmore.reg_mapped.rdata", "Read Data", base.HEX),
    last_seq_num = ProtoField.uint32("canmore.reg_mapped.last_seq_num", "Read Data", base.DEC),
    req_frame = ProtoField.framenum("canmore.reg_mapped.req_framenum", "Request Frame", base.NONE, frametype.REQUEST, 0, 0, "The request this response relates to"),
    resp_frame = ProtoField.framenum("canmore.reg_mapped.resp_framenum", "Response Frame", base.NONE, frametype.RESPONSE, 0, 0, "The response this request relates to")
}
local reg_mapped_e = {
    malformed_pkt = ProtoExpert.new("canmore.reg_mapped.malformed_packet", "Malformed Packet", expert.group.MALFORMED, expert.severity.ERROR),
    unknown_req = ProtoExpert.new("canmore.reg_mapped.unknown_req", "Unknown Request", expert.group.REASSEMBLE, expert.severity.WARN),
    invalid_mode = ProtoExpert.new("canmore.reg_mapped.invalid_mode", "Invalid Mode", expert.group.MALFORMED, expert.severity.ERROR)
}
local reg_mapped_respmap = {}  -- Holds the reconstructed map for requests/responses
local reg_mapped_reqtypemap = {}  -- Holds data for each request so the response can decode it properly
local reg_mapped_reqtype = {
    UNKNOWN = -1,
    READ = 0,
    WRITE = 1,
    BULK_WRITE = 2
}
local reg_mapped_outstanding_reqs = {}  -- Holds the outstanding requests for each client ID
reg_mapped_protocol.fields = reg_mapped_f
reg_mapped_protocol.experts = reg_mapped_e
function reg_mapped_protocol.init()
    -- Need to reset local state when we start a new capture
    reg_mapped_respmap = {}
    reg_mapped_reqtypemap = {}
end
function reg_mapped_protocol.dissector(buffer, pinfo, tree)
    local client_id = tonumber(pinfo.private["canmore_client_id"])

    if (buffer:len() == 0) then
        pinfo.cols.info:set("Reg Mapped Packet [MALFORMED]")
        local subtree = tree:add(reg_mapped_protocol, buffer(), "CANmore Reg Mapped Packet [MALFORMED]")
        subtree:add_proto_expert_info(reg_mapped_e.malformed_pkt, "Empty Packet")
        return 0
    end

    if (pinfo.private["canmore_dir"] == "1") then
        pinfo.cols.protocol:set("Reg Mapped")

        local flag_tvb = buffer:range(0, 1)
        local access_val = bitfield(flag_tvb:uint(), 0, 1)
        local bulk_req_val = bitfield(flag_tvb:uint(), 1, 1)
        local bulk_end_val = bitfield(flag_tvb:uint(), 2, 1)
        local multiword_val = bitfield(flag_tvb:uint(), 3, 1)
        local mode_val = bitfield(flag_tvb:uint(), 5, 3)

        local count_tvb = buffer:range(1, 1)
        local addr_page_tvb = buffer:range(2, 1)
        local addr_offset_tvb = buffer:range(3, 1)

        if ((access_val == 1) and (buffer:len() ~= 8)) or ((access_val == 0) and (buffer:len() ~= 4)) then
            pinfo.cols.info:set("Reg Mapped Request [MALFORMED]")
            local subtree = tree:add(reg_mapped_protocol, flag_tvb, "CANmore Reg Mapped Requet [MALFORMED]")
            subtree:add_proto_expert_info(reg_mapped_e.malformed_pkt, "Invalid Request Length")
            return 1
        end

        -- local heartbeat_data = buffer(0, 1):range(0, 1):uint()

        local tree_title = ""
        if access_val == 1 then
            tree_title = tree_title .. "Write"
        else
            tree_title = tree_title .. "Read"
        end
        tree_title = tree_title .. " Page " .. tostring(addr_page_tvb:uint()) .. ", Offset: " .. tostring(addr_offset_tvb:uint()) .. ", "
        if heartbeat_mode_table[mode_val] ~= nil then
            tree_title = tree_title .. heartbeat_mode_table[mode_val]
        else
            tree_title = tree_title .. "Unknown Mode (" .. tostring(mode_val) .. ")"
        end
        if bulk_req_val == 1 then
            tree_title = tree_title .. ", Seq Count: " .. tostring(count_tvb:uint())
        end

        local subtree = tree:add(reg_mapped_protocol, buffer(), "CANmore Reg Mapped: " .. tree_title)
        local flags_tree = subtree:add(reg_mapped_f.flags, flag_tvb)
        flags_tree:add(reg_mapped_f.access, flag_tvb)
        flags_tree:add(reg_mapped_f.bulk_req, flag_tvb)
        flags_tree:add(reg_mapped_f.bulk_end, flag_tvb)
        flags_tree:add(reg_mapped_f.multiword, flag_tvb)
        flags_tree:add(reg_mapped_f.mode, flag_tvb)
        subtree:add(reg_mapped_f.addr_page, addr_page_tvb)
        subtree:add(reg_mapped_f.addr_offset, addr_offset_tvb)
        if bulk_req_val == 1 then
            subtree:add(reg_mapped_f.count, count_tvb)
        end

        if bulk_end_val == 1 and bulk_req_val ~= 1 then
            subtree:add(reg_mapped_e.invalid_mode, "Cannot request bulk end on non-bulk request")
        elseif bulk_req_val == 1 and access_val ~= 1 then
            subtree:add(reg_mapped_e.invalid_mode, "Cannot perform bulk read request")
        elseif multiword_val == 1 then
            subtree:add(reg_mapped_e.invalid_mode, "Multiword mode not yet supported")
        end

        -- Need to handle inter-frame stuff
        -- Wireshark only gaurentees each dissector is ran in order once, after which any dissector can be called anytime after
        -- We need to use global state variables to keep track of requests/responses
        if bulk_req_val ~= 1 or bulk_end_val == 1 then
            -- Don't store us if we're a bulk request

            local framenum = pinfo.number
            local respframe = reg_mapped_respmap[framenum]
            if respframe == nil then
                -- We haven't processed this packet yet, we need to mark this as the next outstanding request
                -- The next response for this client id will link everything back up
                reg_mapped_outstanding_reqs[client_id] = framenum
                -- Also mark the response frame as processed
                reg_mapped_respmap[framenum] = -1

                -- Store the request type
                if bulk_req_end == 1 then
                    reg_mapped_reqtypemap[framenum] = reg_mapped_reqtype.BULK_READ
                elseif access_val == 1 then
                    reg_mapped_reqtypemap[framenum] = reg_mapped_reqtype.WRITE
                else
                    reg_mapped_reqtypemap[framenum] = reg_mapped_reqtype.READ
                end

            else
                -- We've hit this frame before. Add the response frame link if its valid
                if respframe ~= -1 then
                    local entry = subtree:add(reg_mapped_f.resp_frame, respframe)
                    entry:set_generated(true)
                end
            end
        end

        pinfo.cols.info:set(tree_title)
        return buffer:len()
    else
        pinfo.cols.protocol:set("Reg Mapped")

        -- Link back up the request and responses
        -- Again, need to handle depending on if its the first time we've hit this packet before
        local framenum = pinfo.number
        local reqframe = reg_mapped_respmap[framenum]

        if reqframe == nil then
            -- First time we processed this packet
            -- Get the last request frame
            local last_req_frame = reg_mapped_outstanding_reqs[client_id]
            if last_req_frame ~= nil then
                reqframe = last_req_frame
                reg_mapped_outstanding_reqs[client_id] = -1
            else
                reqframe = -1
            end

            -- Save the frame links into persistant table
            reg_mapped_respmap[framenum] = reqframe
            if reqframe ~= -1 then
                reg_mapped_respmap[reqframe] = framenum
            end
        end

        -- Fetch the request type now that we *should* know the request frame
        -- Fallback to unknown if we couldn't find the request though
        local reqtype = reg_mapped_reqtype.UNKNOWN
        if reqframe ~= -1 then
            reqtype = reg_mapped_reqtypemap[reqframe]
            assert(reqtype ~= nil, "The maps somehow broke")
        end

        -- Fetch the fields depending on the request type
        -- We know the packet is at least 1 byte long (checked above), but perform any additional length checking here
        local result_tvb = buffer:range(0, 1)
        local result_val = result_tvb:uint()
        local seq_num_tvb
        local seq_num_val
        local read_data_tvb
        local read_data_val
        if reqtype == reg_mapped_reqtype.READ then
            if buffer:len() ~= 5 then
                pinfo.cols.info:set("Reg Mapped Response [MALFORMED]")
                local subtree = tree:add(reg_mapped_protocol, result_tvb, "CANmore Reg Mapped Response [MALFORMED]")
                subtree:add_proto_expert_info(reg_mapped_e.malformed_pkt, "Invalid Read Response Length")
                return 1
            end
            read_data_tvb = buffer:range(1, 4)
            read_data_val = read_data_tvb:le_uint()
        elseif reqtype == reg_mapped_reqtype.WRITE then
            if buffer:len() ~= 1 then
                pinfo.cols.info:set("Reg Mapped Response [MALFORMED]")
                local subtree = tree:add(reg_mapped_protocol, result_tvb, "CANmore Reg Mapped Response [MALFORMED]")
                subtree:add_proto_expert_info(reg_mapped_e.malformed_pkt, "Invalid Write Response Length")
                return 1
            end
        elseif reqtype == reg_mapped_reqtype.BULK_WRITE then
            if buffer:len() ~= 2 then
                pinfo.cols.info:set("Reg Mapped Response [MALFORMED]")
                local subtree = tree:add(reg_mapped_protocol, result_tvb, "CANmore Reg Mapped Response [MALFORMED]")
                subtree:add_proto_expert_info(reg_mapped_e.malformed_pkt, "Invalid Bulk Write Response Length")
                return 1
            end
            seq_num_tvb = buffer:range(1, 1)
            seq_num_val = read_data_tvb:uint()
        end

        -- Format the string depending on the request type and result
        local result_str
        if result_val ~= 0 then
            if reg_mapped_result_table[result_val] ~= nil then
                result_str = "Failed: " .. reg_mapped_result_table[result_val]
            else
                result_str = "Unknown Error (" .. tostring(result_val) .. ")"
            end
            if reqtype == reg_mapped_reqtype.BULK_WRITE then
                reuslt_str = result_str .. " on seq num " .. tostring(seq_num_val)
            end
        else
            if reqtype == reg_mapped_reqtype.READ then
                result_str = "Read Success"
            elseif reqtype == reg_mapped_reqtype.WRITE then
                result_str = "Write Success"
            elseif reqtype == reg_mapped_reqtype.BULK_WRITE then
                result_str = "Bulk Write Success (" .. tostring(seq_num_val) .. " frames)"
            else -- UNKNOWN type
                result_str = "Result Success"
            end
        end

        if reqframe == -1 then
            result_str = result_str .. " [MISSING REQUEST]"
        end

        -- Construct the tree
        local subtree = tree:add(reg_mapped_protocol, buffer(), "CANmore Reg Mapped: " .. result_str)
        subtree:add_le(reg_mapped_f.result, result_tvb)

        if reqtype == reg_mapped_reqtype.READ then
            subtree:add_le(reg_mapped_f.read_data, read_data_tvb)
        end
        if reqtype == reg_mapped_reqtype.BULK_WRITE then
            subtree:add_le(reg_mapped_f.last_seq_num, seq_num_tvb)
        end

        if reqframe ~= -1 then
            local entry = subtree:add(reg_mapped_f.req_frame, reqframe)
            entry:set_generated(true)
        else
            subtree:add_proto_expert_info(reg_mapped_e.unknown_req)
        end

        pinfo.cols.info:set(result_str)

        return buffer:len()
    end
end

-- ========================================
-- CANmore CRC18 Calculation
-- ========================================

local CANMORECRC18Lookup = {
	0x00000, 0x23979, 0x24b8b, 0x072f2, 0x2ae6f, 0x09716, 0x0e5e4, 0x2dc9d, 0x365a7, 0x15cde, 0x12e2c, 0x31755, 0x1cbc8,
    0x3f2b1, 0x38043, 0x1b93a, 0x0f237, 0x2cb4e, 0x2b9bc, 0x080c5, 0x25c58, 0x06521, 0x017d3, 0x22eaa, 0x39790, 0x1aee9,
    0x1dc1b, 0x3e562, 0x139ff, 0x30086, 0x37274, 0x14b0d, 0x1e46e, 0x3dd17, 0x3afe5, 0x1969c, 0x34a01, 0x17378, 0x1018a,
    0x338f3, 0x281c9, 0x0b8b0, 0x0ca42, 0x2f33b, 0x02fa6, 0x216df, 0x2642d, 0x05d54, 0x11659, 0x32f20, 0x35dd2, 0x164ab,
    0x3b836, 0x1814f, 0x1f3bd, 0x3cac4, 0x273fe, 0x04a87, 0x03875, 0x2010c, 0x0dd91, 0x2e4e8, 0x2961a, 0x0af63, 0x3c8dc,
    0x1f1a5, 0x18357, 0x3ba2e, 0x166b3, 0x35fca, 0x32d38, 0x11441, 0x0ad7b, 0x29402, 0x2e6f0, 0x0df89, 0x20314, 0x03a6d,
    0x0489f, 0x271e6, 0x33aeb, 0x10392, 0x17160, 0x34819, 0x19484, 0x3adfd, 0x3df0f, 0x1e676, 0x05f4c, 0x26635, 0x214c7,
    0x02dbe, 0x2f123, 0x0c85a, 0x0baa8, 0x283d1, 0x22cb2, 0x015cb, 0x06739, 0x25e40, 0x082dd, 0x2bba4, 0x2c956, 0x0f02f,
    0x14915, 0x3706c, 0x3029e, 0x13be7, 0x3e77a, 0x1de03, 0x1acf1, 0x39588, 0x2de85, 0x0e7fc, 0x0950e, 0x2ac77, 0x070ea,
    0x24993, 0x23b61, 0x00218, 0x1bb22, 0x3825b, 0x3f0a9, 0x1c9d0, 0x3154d, 0x12c34, 0x15ec6, 0x367bf, 0x1a8c1, 0x391b8,
    0x3e34a, 0x1da33, 0x306ae, 0x13fd7, 0x14d25, 0x3745c, 0x2cd66, 0x0f41f, 0x086ed, 0x2bf94, 0x06309, 0x25a70, 0x22882,
    0x011fb, 0x15af6, 0x3638f, 0x3117d, 0x12804, 0x3f499, 0x1cde0, 0x1bf12, 0x3866b, 0x23f51, 0x00628, 0x074da, 0x24da3,
    0x0913e, 0x2a847, 0x2dab5, 0x0e3cc, 0x04caf, 0x275d6, 0x20724, 0x03e5d, 0x2e2c0, 0x0dbb9, 0x0a94b, 0x29032, 0x32908,
    0x11071, 0x16283, 0x35bfa, 0x18767, 0x3be1e, 0x3ccec, 0x1f595, 0x0be98, 0x287e1, 0x2f513, 0x0cc6a, 0x210f7, 0x0298e,
    0x05b7c, 0x26205, 0x3db3f, 0x1e246, 0x190b4, 0x3a9cd, 0x17550, 0x34c29, 0x33edb, 0x107a2, 0x2601d, 0x05964, 0x02b96,
    0x212ef, 0x0ce72, 0x2f70b, 0x285f9, 0x0bc80, 0x105ba, 0x33cc3, 0x34e31, 0x17748, 0x3abd5, 0x192ac, 0x1e05e, 0x3d927,
    0x2922a, 0x0ab53, 0x0d9a1, 0x2e0d8, 0x03c45, 0x2053c, 0x277ce, 0x04eb7, 0x1f78d, 0x3cef4, 0x3bc06, 0x1857f, 0x359e2,
    0x1609b, 0x11269, 0x32b10, 0x38473, 0x1bd0a, 0x1cff8, 0x3f681, 0x12a1c, 0x31365, 0x36197, 0x158ee, 0x0e1d4, 0x2d8ad,
    0x2aa5f, 0x09326, 0x24fbb, 0x076c2, 0x00430, 0x23d49, 0x37644, 0x14f3d, 0x13dcf, 0x304b6, 0x1d82b, 0x3e152, 0x393a0,
    0x1aad9, 0x013e3, 0x22a9a, 0x25868, 0x06111, 0x2bd8c, 0x084f5, 0x0f607, 0x2cf7e
}

function canmore_crc18(bytes)
	local crc = 0x3ffff
	for i=0,(bytes:len() - 1) do
		local b = bytes:get_index(i)
		crc = bit32.bxor(bit32.lshift(crc, 8), CANMORECRC18Lookup[bit32.band(bit32.bxor(bit32.rshift(crc, 10), b), 0xff) + 1])
	end
	return bit32.band(crc, 0x3ffff)
end

-- ========================================
-- CANmore Message Re-assembler
-- ========================================

local canmore_msg_protocol = Proto("canmore.msg",  "CANmore Message Frame")
local canmore_msg_f = {
    msg_type = ProtoField.string("canmore.msg.type", "Type"),
    crc = ProtoField.uint32("canmore.msg.crc", "Checksum", base.HEX),
    crc_computed = ProtoField.uint32("canmore.msg.crc_computed", "Computed Checksum", base.HEX),
    fragment = ProtoField.bytes("canmore.msg.fragment", "Fragment Data", base.NONE),
    fragment_len = ProtoField.uint32("canmore.msg.fragment_len", "Fragment Length", base.DEC),
    msg_idx = ProtoField.uint32("canmore.msg.index", "Conversation Index", base.DEC),
    reassembled_in = ProtoField.framenum("canmore.msg.reassembled_in", "Reassembled PDU in Frame")
}
local canmore_msg_e = {
    reassembly_fail = ProtoExpert.new("canmore.reg_mapped.reassembly_fail", "Failed to Reassemble", expert.group.REASSEMBLE, expert.severity.ERROR),
    standalone_warn = ProtoExpert.new("canmore.reg_mapped.reassembly_fail", "Could not find complete message sequence for this packet", expert.group.REASSEMBLE, expert.severity.WARN),
}
canmore_msg_protocol.fields = canmore_msg_f
canmore_msg_protocol.experts = canmore_msg_e
-- This is unfortunately the best way to reassemble packets:
-- https://osqa-ask.wireshark.org/questions/31670/piecing-together-multipart-udp-messages-with-lua-wireshark-dissector/
-- The pinfo.desegment_offset and pinfo.desegment_len appear to only be used for dissecting streams rather than frames
-- Instead, we need to copy all the contents into a table, addressed by the start packet frame number. On the first run
-- through (as wireshark only gaurentees the dissectors to run in order ONCE at startup), we will store the frame id
-- in a temporary table which will contain the last transmitted start frame num for a given client id & direction. The
-- start frame will also add an entry into the bytearray table to hold the reassembled PDU.
-- On the first dissection of every non-start frame, it will use the last start frame num as its assumed start frame
-- from that table mentioned before. From there, it will append its contents to the bytearray table. On the last frame
-- it will use this table to pull the complete PDU out. In the event an error occurs during dissection (e.g. invalid
-- sequence number), the temporary table will be invalidated to ensure that another start packet must occur before
-- reassmebly will restart.
local canmore_msg_reassembled_pdus = {}  -- Holds bytearrays of the reassembled PDUs [key is message index]
local canmore_msg_frame_array = {}       -- Holds all frames that are part of a given message index [key is message index]
local canmore_msg_last_frame_lookup = {} -- Holds the last frame for every message. Can also be used to see if the message is complete
local canmore_msg_crc18_calc = {}        -- Holds the computed CRC18 for each message
local canmore_msg_msg_idx_lookup = {}    -- Holds a lookup for each processed frame to its corresponding universal message index [key is framenum]
local canmore_msg_error_lookup = {}      -- Holds error message found during the first pass for a given frame [key is framenum]
local canmore_msg_active_msgs = {}       -- Holds the message index for each client/direction transfer in flight (only used on first pass)
local canmore_msg_active_next_seq = {}   -- Holds the next expected sequence number for each transfer in flight (only used on first pass)
local canmore_msg_next_msg_idx = 1       -- The next available message index to be used (only used on first pass)

function canmore_msg_protocol.init()
    -- Reset state when file is reloaded
    canmore_msg_reassembled_pdus = {}
    canmore_msg_frame_array = {}
    canmore_msg_last_frame_lookup = {}
    canmore_msg_crc18_calc = {}
    canmore_msg_msg_idx_lookup = {}
    canmore_msg_error_lookup = {}
    canmore_msg_active_msgs = {}
    canmore_msg_active_next_seq = {}
    canmore_msg_next_msg_idx = 1
end
function canmore_msg_protocol.dissector(buffer, pinfo, tree)
    local client_id = tonumber(pinfo.private["canmore_client_id"])
    local dir_val = tonumber(pinfo.private["canmore_dir"])
    local seq_num = tonumber(pinfo.private["canmore_noc"])
    local id_extra_val = tonumber(pinfo.private["canmore_extra"])
    local can_id_tvb = can_id_field().range

    pinfo.cols.protocol:set("Message Frame")

    local is_first = (seq_num == 0)
    local is_last = (id_extra_val ~= nil)

    -- Do first time processing if we haven't reached this packet yet
    local msg_idx
    local framenum = pinfo.number
    if not pinfo.visited then
        -- Need a unique stream identifier to refer to active transfers
        local stream_id = (client_id * 2)  + dir_val
        if is_first then
            -- We're the first frame, grab create a new message index for this message sequence
            msg_idx = canmore_msg_next_msg_idx
            canmore_msg_reassembled_pdus[msg_idx] = ByteArray.new()
            canmore_msg_frame_array[msg_idx] = {}
            canmore_msg_active_msgs[stream_id] = msg_idx
            canmore_msg_active_next_seq[stream_id] = 0
            canmore_msg_next_msg_idx = canmore_msg_next_msg_idx + 1
        else
            -- Not the first frame, we need to grab whatever the current active message index for this stream id
            msg_idx = canmore_msg_active_msgs[stream_id]
        end

        -- We can only do the next steps if there's an active message transfer in flight
        -- This isn't always the case, in the event it starts in the middle of a transmission, or we get out of order
        if msg_idx ~= nil then
            -- Verify that the sequence number is what we expect
            local exp_next_seq = canmore_msg_active_next_seq[stream_id]
            if exp_next_seq == seq_num then
                -- This is the next packet in the sequence, append the contents to the reassembled array
                canmore_msg_reassembled_pdus[msg_idx]:append(buffer:bytes())

                -- Do final processing if last item in array
                if is_last then
                    -- Verify the CRC
                    computed_crc = canmore_crc18(canmore_msg_reassembled_pdus[msg_idx])
                    canmore_msg_crc18_calc[msg_idx] = computed_crc
                    if computed_crc == id_extra_val then
                        -- Record this as the last frame, marking the message as complete
                        canmore_msg_last_frame_lookup[msg_idx] = framenum

                    else
                        -- We hit an error, report it
                        canmore_msg_error_lookup[framenum] = "CRC-18 checksum did not match computed value"
                        -- Don't set msg_idx to nil, though, since we did reassemble everything
                    end

                    -- Clear the active transfer, as we either just finished it or failed with an error
                    canmore_msg_active_msgs[stream_id] = nil
                    canmore_msg_active_next_seq[stream_id] = nil
                else
                    exp_next_seq = exp_next_seq + 1
                    if exp_next_seq >= 16 then  -- Handle sequence number rollover
                        exp_next_seq = 0  -- If rollover is disabled later, this should instead raise an error and clear the transfer state
                    end
                    canmore_msg_active_next_seq[stream_id] = exp_next_seq
                end
            else
                -- Sequence did not match next expected
                -- Set the error message that the stream was out of order
                canmore_msg_error_lookup[framenum] = "Sequence number did not match next expected seq num " .. tostring(exp_next_seq)

                -- Clear active transfer since we don't want it to continue reassembling things
                canmore_msg_active_msgs[stream_id] = nil
                canmore_msg_active_next_seq[stream_id] = nil

                -- Also clear the message index, since we found out we aren't actually part of that anymore
                msg_idx = nil
            end
        end

        -- If we get here, and msg_idx is still valid, that means this packet is okay to be part of the message sequence
        -- Assign this frame to this message
        if msg_idx ~= nil then
            canmore_msg_msg_idx_lookup[framenum] = msg_idx
            table.insert(canmore_msg_frame_array[msg_idx], framenum)
        end
    else
        msg_idx = canmore_msg_msg_idx_lookup[framenum]
    end

    -- Fetch error message if its present
    local err_msg = canmore_msg_error_lookup[framenum]

    -- Compute the string titles
    local subtree_title
    local msg_type
    if is_first and is_last then
        subtree_title = "Single Frame Packet"
        msg_type = "Single"
    elseif is_first then
        subtree_title = "Start Frame"
        msg_type = "Start"
    elseif is_last then
        subtree_title = "Final Frame; Total Frames=" .. tostring(seq_num + 1)
        msg_type = "End"
    else
        subtree_title = "Continuation Frame; Seq Num=" .. tostring(seq_num)
        msg_type = "Middle Fragment"
    end

    if msg_idx == nil then
        subtree_title = subtree_title .. " [STANDALONE FRAGMENT]"
    end


    if msg_idx ~= nil and not is_last and canmore_msg_last_frame_lookup[msg_idx] ~= nil then
        subtree_title = subtree_title .. " [Part of Reassembled Msg]"
    end

    -- Add all the elements to the tree
    local entry
    local subtree = tree:add(canmore_msg_protocol, buffer(), "CANmore Message: " .. subtree_title)
    if msg_idx ~= nil then
        -- Add message index if we have one
        entry = subtree:add(canmore_msg_f.msg_idx, msg_idx)
        entry:set_generated(true)
    end
    if err_msg ~= nil then
        -- Add error message if present
        subtree:add_proto_expert_info(canmore_msg_e.reassembly_fail, err_msg)
    elseif msg_idx == nil then
        subtree:add_proto_expert_info(canmore_msg_e.standalone_warn)
    end

    if is_last then
        -- Add CRC if we're the last message
        subtree:add(canmore_msg_f.crc, can_id_tvb, id_extra_val)
    end
    if is_last and msg_idx ~= nil then
        -- Add computed CRC if we're part of a full message
        entry = subtree:add(canmore_msg_f.crc_computed, canmore_msg_crc18_calc[msg_idx])
        entry:set_generated(true)
    end
    entry = subtree:add(canmore_msg_f.msg_type, msg_type)
    entry:set_generated(true)
    entry = subtree:add(canmore_msg_f.fragment_len, buffer:len())
    entry:set_generated(true)
    subtree:add(canmore_msg_f.fragment, buffer())
    if not is_last and msg_idx ~= nil then
        -- We don't reassemble it here, mark which frame it is assembled in
        local reassembled_in = canmore_msg_last_frame_lookup[msg_idx]
        if reassembled_in ~= nil then
            -- Only add if we actually have a last packet this is assembled in
            entry = subtree:add(canmore_msg_f.reassembled_in, reassembled_in)
            entry:set_generated(true)
        end
    end

    -- Call the reassembled decoder if we are a complete packet
    if is_last and msg_idx ~= nil then
        local reassembled_tvb = ByteArray.tvb(canmore_msg_reassembled_pdus[msg_idx], "Reassembled Msg")
        Dissector.get("canmore.msg.reassembled"):call(reassembled_tvb, pinfo, tree)
    end

    pinfo.cols.info:set(subtree_title)

    return buffer:len()

end


-- ========================================
-- CANmore Message Decoder
-- ========================================
local canmore_msg_decoder_protocol = Proto("canmore.msg.reassembled",  "CANmore Message Reassmbled")
local canmore_msg_decoder_f = {
    fragment = ProtoField.framenum("canmore.msg.reassembled.fragment", "Reassembled From Frame"),
    count = ProtoField.uint32("canmore.msg.reassembled.frame_count", "Frame Count", base.DEC),
    len = ProtoField.uint32("canmore.msg.reassembled.length", "Reassembled Length", base.DEC),
    data = ProtoField.bytes("canmore.msg.reassembled.data", "Reassembled Data", base.NONE)
}
canmore_msg_decoder_protocol.fields = canmore_msg_decoder_f
function canmore_msg_decoder_protocol.dissector(buffer, pinfo, tree)
    -- Fetch the data for this specific frame
    -- This is only called if msg_idx is valid, so we don't have to do any extra checks for that
    local framenum = pinfo.number
    local msg_idx = canmore_msg_msg_idx_lookup[framenum]
    local reassembled_len = buffer:len()

    local num_segments = #canmore_msg_frame_array[msg_idx]
    local tree_title = tostring(num_segments) .. " Reassembled Message Frames (" .. tostring(reassembled_len) .. " bytes): "
    local first = true
    for _, framenum in ipairs(canmore_msg_frame_array[msg_idx]) do
        if not first then
            tree_title = tree_title .. ", "
        else
            first = false
        end
        tree_title = tree_title .. "#" .. framenum
    end

    local entry
    local subtree = tree:add(canmore_msg_decoder_protocol, buffer(), tree_title)
    subtree:set_generated(true)
    for _, framenum in ipairs(canmore_msg_frame_array[msg_idx]) do
        entry = subtree:add(canmore_msg_decoder_f.fragment, framenum)
        entry:set_generated(true)
    end
    entry = subtree:add(canmore_msg_decoder_f.count, num_segments)
    entry:set_generated(true)
    entry = subtree:add(canmore_msg_decoder_f.len, reassembled_len)
    entry:set_generated(true)
    entry = subtree:add(canmore_msg_decoder_f.data, buffer())
    entry:set_generated(true)

    -- TODO: Switch to protobuf when ready
    local xrce_dis = Dissector.get("xrce-dds")
    if xrce_dis ~= nil then
        xrce_dis:call(buffer, pinfo, tree)
    end

end
