const express = require("express")
const path = require("path")
const bodyParser = require("body-parser")
const net = require("net")

const app = express()
const PORT = 4200
const REFEREE_ADDRESS = "ws://192.168.3.220:8111"
const ROBOT_NAME = "mu"

app.use(express.static(path.join(__dirname, "public")))
app.use(bodyParser.json())

const websocket = require("ws");

const robot = net.createConnection({
    port: 6969,
    onread: {
        buffer: Buffer.alloc(4 * 1024),
        callback: (nread, buf) => { }
    }
})

const wss = new websocket.WebSocket(REFEREE_ADDRESS)

wss.on('error', (data) => {
    console.error(`Couldn't connect with Referee`)
})

robotSpeeds = [0, 0, 0, 0]
manual = true

wss.on('message', (data) => {
    try {
        req = JSON.parse(data.toString())
        console.log(req)
        if (req.signal && req.signal === 'start') {
            if (req.targets && Array.isArray(req.targets)) {
                i = req.targets.indexOf(ROBOT_NAME)
                if (i != -1 && req.baskets && Array.isArray(req.baskets) && req.baskets.length > i) {
                    const basket = req.baskets[i];
                    robot.write(JSON.stringify({ mode: "auto", basket, speeds: [0, 0, 0, 0] }))
                    manual = false
                } else {
                    throw "Not about us / bad baskets"
                }
            }
        } else if (req.signal && req.signal === 'stop') {
            if (req.targets && req.targets.includes(ROBOT_NAME)) {
                robot.write(JSON.stringify({ mode: "stop" }))
                manual = true
                robotSpeeds = [0, 0, 0, 0]
            }
        } else
            throw "Illegal signal"
    } catch {
        console.error(`Didn't understand referee command, ${data.toString()}`)
    }
})

setInterval(() => {
    if (manual) {
        robot.write(JSON.stringify({ mode: 'manual', speeds: robotSpeeds }))
    }
}, 100)


setSpeeds = (speed, clicked) => {
    switch (clicked) {
        case "forward":
            robotSpeeds[0] = 0
            robotSpeeds[1] = speed
            robotSpeeds[2] = -speed
            break
        case "backward":
            robotSpeeds[0] = 0
            robotSpeeds[1] = -speed
            robotSpeeds[2] = speed
            break
        case "right":
            robotSpeeds[0] = speed
            robotSpeeds[1] = -Math.floor(speed / 2)
            robotSpeeds[2] = -Math.floor(speed / 2)
            break
        case "left":
            robotSpeeds[0] = -speed
            robotSpeeds[1] = Math.floor(speed / 2)
            robotSpeeds[2] = Math.floor(speed / 2)
            break
        case "spinLeft":
            robotSpeeds[0] = speed
            robotSpeeds[1] = speed
            robotSpeeds[2] = speed
            break
        case "spinRight":
            robotSpeeds[0] = -speed
            robotSpeeds[1] = -speed
            robotSpeeds[2] = -speed
            break
    }
}


app.put("/api", (req, res) => {
    if (req.body && req.body.clicked && (req.body.speed != undefined)) {
        const speed = new Number(req.body.speed)
        if (!Number.isNaN(speed)) {
            switch (req.body.clicked) {
                case "forward":
                case "backward":
                case "left":
                case "spinLeft":
                case "spinRight":
                case "right":
                    if (0 < speed < 50) {
                        setSpeeds(speed, req.body.clicked)
                        manual = true
                        return res.status(200).json({ message: "OK" })
                    } else {
                        return res.status(400).json({ message: "Bad speed" })
                    }
                case "throw":
                    if (0 < speed < 2000) {
                        robotSpeeds[3] = speed
                        manual = true
                        return res.status(200).json({ message: "OK" })
                    } else {
                        return res.status(400).json({ message: "Bad speed" })
                    }
                case "start":
                    manual = false
                    robot.write(JSON.stringify({ mode: 'auto', speeds: [0, 0, 0, 0], basket: 'magenta' }))
                    return res.status(200).json({ message: "OK" })
                case "stop":
                    robotSpeeds = [0, 0, 0, robotSpeeds[3]]
                    manual = true
                    return res.status(200).json({ message: "OK" })
                default:
                    return res.status(400).json({ message: "Bad clicked" })
            }
        } else return res.status(400).json({ message: "Bad request" })
    }
    else return res.status(400).json({ message: "Bad request" })
})

app.listen(PORT, 'localhost', () => {
    console.log(`Server listening on port ${PORT}`)
})