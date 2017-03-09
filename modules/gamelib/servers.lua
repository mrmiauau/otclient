
function printMessage(proto, msg)
  local buffer = msg:getBuffer()
  local str = ''
  for i = 1, msg:getMessageSize() do
    str = str .. string.format("%02X ", string.byte(buffer, i))
  end
  print(str)
end

function printMessageAndReplaceWithPing(proto, msg)
  printMessage(proto, msg)
  msg:setWritePos(8)
  msg:addU8(0x1E)
end

specialServers = {
  ['localhost'] = {version = 854, ip='127.0.0.1', port=7171},
  --['special server'] = {version = 772, ip="localhost", port = 7171, proto = 0x1234, hookPre = printMessage, hookPost = printMessage, things = 'path/to/things', rsa="..."},
}