-- @docclass
ProtocolLoginHttp = extends(Protocol, "ProtocolLoginHttp")

local http = require("socket.http")
local ltn12 = require("ltn12")

function ProtocolLoginHttp:setUrl(url) 
  self.url = url
end

function ProtocolLoginHttp:login(host, port, accountName, accountPassword, authenticatorToken, stayLogged)
  if string.len(self.url) == 0 then
    signalcall(self.onLoginError, self, tr("You must enter a valid login url."))
    return
  end

  self.accountName = accountName
  self.accountPassword = accountPassword
  self.authenticatorToken = authenticatorToken
  self.stayLogged = stayLogged

  local payload = '{"accountname":"' .. accountName ..'","password":"'.. accountPassword ..'","stayloggedin":'..(stayLogged and 'true' or 'false') ..',"type":"login"}'
  local response_body = { }

  local res, code, response_headers, status = http.request({
    url = self.url,
    method = "POST",
    headers =
    {
      ["Content-Type"] = "application/json",
      ["Content-Length"] = payload:len(),
      ["User-Agent"] = "Mozilla/5.0"
    },
    source = ltn12.source.string(payload),
    sink = ltn12.sink.table(response_body)
  })
  if code ~= 200 then 
    signalcall(self.onLoginError, self, "Server returned login code " .. tostring(code))
    return
  end

  local data = table.concat(response_body)
  local err = data:match("errorMessage\":\"([^\"]+)")
  if err then
    signalcall(self.onLoginError, self, err)
    return
  end

  local sessionkey = data:match("sessionkey\":\"([^\"]+)"):gsub('\\n', '\n')
  if not sessionkey then
    signalcall(self.onLoginError, self, 'No sessionKey found in login reply!')
    return
  end

  signalcall(self.onSessionKey, self, sessionkey)
  self:parseCharacterList(data)
end

function ProtocolLoginHttp:parseCharacterList(data)
  local worlds_ = data:match('worlds":%[([^%]]+)%]')
  local worlds = {}
  for world_ in worlds_:gmatch('%{([^%}]+)%}') do 
    local world = {}
    local t = world_:split(',')
    for i = 1, #t do
      local vars = t[i]:split(':')
      local key = vars[1]:sub(2, vars[1]:len()-1)
      local val = vars[2]:sub(2, vars[2]:len()-1)
      -- print(key .. ' => ' .. vars[2])
      if key == 'id' then
        worlds[vars[2]] = world
      elseif key == 'name' then
        world.worldName = val
      elseif key == 'externaladdress' then
        world.worldIp = val
      elseif key == 'externalport' then
        world.worldPort = tonumber(vars[2])
      elseif key == 'previewstate' then
        world.previewState = tonumber(vars[2])
      end
    end
  end
  local characters_ = data:match('characters":%[([^%]]+)%]')
  local characters = {}
  for char_ in characters_:gmatch('%{([^%}]+)%}') do 
    local character = {}
    local t = char_:split(',')

    local name
    for i = 1, #t do
      local vars = t[i]:split(':')
      local key = vars[1]:sub(2, vars[1]:len()-1)
      if key == 'worldid' then
        character = table.copy(worlds[vars[2]])
      elseif key == 'name' then
        name = vars[2]:sub(2, vars[2]:len()-1)
      end
    end
    character.name = name
    table.insert(characters, character)
  end

  local account = {}
  account.premDays = 0
  signalcall(self.onCharacterList, self, characters, account)
end
