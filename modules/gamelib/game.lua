function g_game.getRsa()
  return G.currentRsa
end

function g_game.findPlayerItem(itemId, subType)
    local localPlayer = g_game.getLocalPlayer()
    if localPlayer then
        for slot = InventorySlotFirst, InventorySlotLast do
            local item = localPlayer:getInventoryItem(slot)
            if item and item:getId() == itemId and (subType == -1 or item:getSubType() == subType) then
                return item
            end
        end
    end

    return g_game.findItemInContainers(itemId, subType)
end

function g_game.chooseRsa(host)
  if G.currentRsa ~= CIPSOFT_RSA and G.currentRsa ~= OTSERV_RSA then return end
  if host:ends('.tibia.com') or host:ends('.cipsoft.com') then
    g_game.setRsa(CIPSOFT_RSA)

    if g_app.getOs() == 'windows' then
      g_game.setCustomOs(OsTypes.Windows)
    else
      g_game.setCustomOs(OsTypes.Linux)
    end
  else
    if G.currentRsa == CIPSOFT_RSA then
      g_game.setCustomOs(-1)
    end
    g_game.setRsa(OTSERV_RSA)
  end

  -- Hack fix to resolve some 760 login issues
  if g_game.getClientVersion() <= 760 then
    g_game.setCustomOs(2)
  end
end

function g_game.setRsa(rsa, e)
  e = e or '65537'
  g_crypt.rsaSetPublicKey(rsa, e)
  G.currentRsa = rsa
end

function g_game.isOfficialTibia()
  return G.currentRsa == CIPSOFT_RSA
end

function g_game.getSupportedClients()
  local clients = {
    854, 772
  }
  for k, v in pairs(specialServers) do
    table.insert(clients, k)
  end
  return clients
end

-- The client version and protocol version where
-- unsynchronized for some releases, not sure if this
-- will be the normal standard.

-- Client Version: Publicly given version when
-- downloading Cipsoft client.

-- Protocol Version: Previously was the same as
-- the client version, but was unsychronized in some
-- releases, now it needs to be verified and added here
-- if it does not match the client version.

-- Reason for defining both: The server now requires a
-- Client version and Protocol version from the client.

-- Important: Use getClientVersion for specific protocol
-- features to ensure we are using the proper version.

function g_game.getClientProtocolVersion(client)
  local clients = {
    [980] = 971,
    [981] = 973,
    [982] = 974,
    [983] = 975,
    [984] = 976,
    [985] = 977,
    [986] = 978,
    [1001] = 979,
    [1002] = 980
  }
  return clients[client] or client
end

if not G.currentRsa then
  g_game.setRsa(OTSERV_RSA)
end
