const express = require("express")
const path = require("path")
const bodyParser = require("body-parser")
const net = require("net")

const app = express()
const PORT = 4200
const REFEREE_ADDRESS = "ws://localhost:8222"
const ROBOT_NAME = "My"

app.use(express.static(path.join(__dirname, "public")))
app.use(bodyParser.json())
app.use((req, res, next) => {
    console.log(req.originalUrl)
    console.log(req.body)
    next()
})

const websocket = require("ws");

const robot = net.createConnection({
    port: 6969,
    onread: {
        buffer: Buffer.alloc(4 * 1024),
        callback: (nread, buf) => {}
    }
})

const wss = new websocket.WebSocket(REFEREE_ADDRESS)

wss.on('error', (data) => {
    console.error(`Couldn't connect with Referee`)
})

wss.on('message', (data) => {
    try {
        req = JSON.parse(data.toString())
        console.log(req)
        if (req.signal && req.signal === 'start') {
            if (req.targets && Array.isArray(req.targets)) {
                i = req.targets.indexOf(ROBOT_NAME)
                if (i != -1 && req.baskets && Array.isArray(req.baskets) && req.baskets.length > i) {
                    const basket = req.baskets[i];
                    robot.write(JSON.stringify({ mode: "start", basket }))
                } else {
                    throw "Not about us / bad baskets"
                }
            }
        } else if (req.signal && req.signal === 'stop') {
            if (req.targets && req.targets.includes(ROBOT_NAME)) {
                robot.write(JSON.stringify({ mode: "stop" }))
            }
        } else 
        throw "Illegal signal"
    } catch {
        console.error(`Didn't understand referee command, ${data.toString()}`)
    }
})


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
                        robot.write(JSON.stringify({ mode: req.body.clicked, speed }))
                        return res.status(200).json({ message: "OK" })
                    } else {
                        return res.status(400).json({ message: "Bad speed" })
                    }
                case "throw":
                    if (0 < speed < 2000) {
                        robot.write(JSON.stringify({ mode: req.body.clicked, speed }))
                        return res.status(200).json({ message: "OK" })
                    } else {
                        return res.status(400).json({ message: "Bad speed" })
                    }
                case "start":
                case "stop":
                    robot.write(JSON.stringify({ mode: req.body.clicked, speed }))
                    return res.status(200).json({ message: "OK" })
                default:
                    return res.status(400).json({ message: "Bad clicked" })
            }
        } else return res.status(400).json({ message: "Bad request" })
    }
    else return res.status(400).json({ message: "Bad request" })
})

app.listen(PORT, () => {
    console.log(`Server listening on port ${PORT}`)
})