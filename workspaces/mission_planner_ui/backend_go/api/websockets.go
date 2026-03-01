package api

import (
	"log"
	"net/http"
	"time"

	"mission_planner_backend/auth"
	"mission_planner_backend/models"
	"mission_planner_backend/mqttclient"

	"github.com/gin-gonic/gin"
	"github.com/gorilla/websocket"
)

var upgrader = websocket.Upgrader{
	CheckOrigin: func(r *http.Request) bool {
		// Allow requests from React frontend on any IP (e.g. 192.168.1.93)
		return true
	},
}

// wsAuth handles grabbing the token from query param and validating
func wsAuth(c *gin.Context) bool {
	tokenStr := c.Query("token")
	if tokenStr == "" {
		c.AbortWithStatus(http.StatusForbidden)
		return false
	}
	_, err := auth.ValidateToken(tokenStr)
	if err != nil {
		c.AbortWithStatus(http.StatusForbidden)
		return false
	}
	return true
}

// WsPoints streams voxel points array buffer
func WsPoints(c *gin.Context) {
	if !wsAuth(c) {
		return
	}
	conn, err := upgrader.Upgrade(c.Writer, c.Request, nil)
	if err != nil {
		log.Println("WS Upgrade Error:", err)
		return
	}
	defer conn.Close()

	for {
		time.Sleep(100 * time.Millisecond)
		mqttclient.StateMutex.RLock()
		payload := mqttclient.PointsMsg
		mqttclient.StateMutex.RUnlock()

		if len(payload) > 0 {
			if err := conn.WriteMessage(websocket.BinaryMessage, payload); err != nil {
				return
			}
		}
	}
}

// WsTF streams TF JSON and Path JSON
func WsTF(c *gin.Context) {
	if !wsAuth(c) {
		return
	}
	conn, err := upgrader.Upgrade(c.Writer, c.Request, nil)
	if err != nil {
		log.Println("WS Upgrade Error:", err)
		return
	}
	defer conn.Close()

	for {
		time.Sleep(50 * time.Millisecond) // 20Hz Update
		mqttclient.StateMutex.RLock()

		tfs := mqttclient.TfsMsg
		if tfs == nil {
			tfs = make(map[string]models.TF) // To avoid nil map marshaling to null
		}
		path := mqttclient.PathMsg
		if path == nil {
			path = make([]models.PathPoint, 0)
		}

		payload := models.WsTFPayload{
			TFs:  tfs,
			Path: path,
		}
		mqttclient.StateMutex.RUnlock()

		if err := conn.WriteJSON(payload); err != nil {
			return
		}
	}
}

// WsVideo streams Base64 JPEG strings
func WsVideo(c *gin.Context) {
	if !wsAuth(c) {
		return
	}
	conn, err := upgrader.Upgrade(c.Writer, c.Request, nil)
	if err != nil {
		log.Println("WS Upgrade Error:", err)
		return
	}
	defer conn.Close()

	for {
		mqttclient.StateMutex.RLock()
		video := mqttclient.VideoMsg
		mqttclient.StateMutex.RUnlock()

		if video != "" {
			if err := conn.WriteMessage(websocket.TextMessage, []byte(video)); err != nil {
				return
			}
		}
		time.Sleep(100 * time.Millisecond) // 10 FPS
	}
}
