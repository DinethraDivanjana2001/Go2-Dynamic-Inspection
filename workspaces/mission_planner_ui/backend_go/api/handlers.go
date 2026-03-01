package api

import (
	"encoding/json"
	"net/http"
	"os"

	"mission_planner_backend/auth"
	"mission_planner_backend/database"
	"mission_planner_backend/models"
	"mission_planner_backend/mqttclient"

	"github.com/gin-gonic/gin"
)

// AuthMiddleware protects routes requiring JWT
func AuthMiddleware() gin.HandlerFunc {
	return func(c *gin.Context) {
		tokenString := c.GetHeader("Authorization")
		if tokenString == "" {
			c.AbortWithStatusJSON(http.StatusUnauthorized, gin.H{"error": "Authorization header required"})
			return
		}

		if len(tokenString) > 7 && tokenString[:7] == "Bearer " {
			tokenString = tokenString[7:]
		}

		username, err := auth.ValidateToken(tokenString)
		if err != nil {
			c.AbortWithStatusJSON(http.StatusUnauthorized, gin.H{"error": "Invalid token"})
			return
		}

		c.Set("username", username)
		c.Next()
	}
}

// Register creates a new user
func Register(c *gin.Context) {
	var userReq models.UserCreate
	if err := c.ShouldBindJSON(&userReq); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
		return
	}

	// Check if exists
	var existing models.User
	if err := database.DB.Where("username = ?", userReq.Username).First(&existing).Error; err == nil {
		c.JSON(http.StatusBadRequest, gin.H{"detail": "Username already registered"})
		return
	}

	hashed, err := auth.HashPassword(userReq.Password)
	if err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Failed to hash password"})
		return
	}

	user := models.User{
		Username:       userReq.Username,
		HashedPassword: hashed,
	}

	if err := database.DB.Create(&user).Error; err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Database error"})
		return
	}

	c.JSON(http.StatusOK, gin.H{"message": "User created successfully"})
}

// Login creates a JWT
func Login(c *gin.Context) {
	// Support OAuth2PasswordRequestForm mimicking FastAPI (x-www-form-urlencoded)
	var form struct {
		Username string `form:"username" binding:"required"`
		Password string `form:"password" binding:"required"`
	}

	if err := c.ShouldBind(&form); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"detail": "Invalid form data"})
		return
	}

	var user models.User
	if err := database.DB.Where("username = ?", form.Username).First(&user).Error; err != nil {
		c.JSON(http.StatusUnauthorized, gin.H{"detail": "Incorrect username or password"})
		return
	}

	if !auth.VerifyPassword(form.Password, user.HashedPassword) {
		c.JSON(http.StatusUnauthorized, gin.H{"detail": "Incorrect username or password"})
		return
	}

	token, err := auth.CreateAccessToken(user.Username)
	if err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Failed to generate token"})
		return
	}

	c.JSON(http.StatusOK, gin.H{
		"access_token": token,
		"token_type":   "bearer",
	})
}

// GetProfile returns the current user profile (including base64 profile pic)
func GetProfile(c *gin.Context) {
	username, _ := c.Get("username")

	var user models.User
	if err := database.DB.Where("username = ?", username).First(&user).Error; err != nil {
		c.JSON(http.StatusNotFound, gin.H{"error": "User not found"})
		return
	}

	c.JSON(http.StatusOK, models.UserProfile{
		Username:    user.Username,
		DisplayName: user.DisplayName,
		ProfilePic:  user.ProfilePic,
	})
}

// UpdateProfile updates user profile settings
func UpdateProfile(c *gin.Context) {
	username, _ := c.Get("username")

	var req models.UserProfileUpdate
	if err := c.ShouldBindJSON(&req); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
		return
	}

	var user models.User
	if err := database.DB.Where("username = ?", username).First(&user).Error; err != nil {
		c.JSON(http.StatusNotFound, gin.H{"error": "User not found"})
		return
	}

	if req.ProfilePic != nil {
		user.ProfilePic = *req.ProfilePic
	}
	if req.DisplayName != nil {
		user.DisplayName = *req.DisplayName
	}

	if err := database.DB.Save(&user).Error; err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Failed to update profile"})
		return
	}

	c.JSON(http.StatusOK, gin.H{"message": "Profile updated successfully"})
}

// Navigate sends a goal to MQTT
func Navigate(c *gin.Context) {
	var goal models.GoalRequest
	if err := c.ShouldBindJSON(&goal); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
		return
	}

	if err := mqttclient.PublishNavigate(goal); err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"error": err.Error()})
		return
	}

	c.JSON(http.StatusOK, gin.H{
		"status": "Goal sent to MQTT",
		"target": goal,
	})
}

const waypointsFile = "waypoints.json"

// GetWaypoints reads from JSON config
func GetWaypoints(c *gin.Context) {
	if _, err := os.Stat(waypointsFile); os.IsNotExist(err) {
		c.JSON(http.StatusOK, make([]models.Waypoint, 0))
		return
	}

	file, err := os.ReadFile(waypointsFile)
	if err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Failed to read waypoints"})
		return
	}

	var waypoints []models.Waypoint
	if err := json.Unmarshal(file, &waypoints); err != nil {
		waypoints = make([]models.Waypoint, 0)
	}
	c.JSON(http.StatusOK, waypoints)
}

// SaveWaypoint adds to JSON config
func SaveWaypoint(c *gin.Context) {
	var wp models.Waypoint
	if err := c.ShouldBindJSON(&wp); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
		return
	}

	var waypoints []models.Waypoint
	if file, err := os.ReadFile(waypointsFile); err == nil {
		json.Unmarshal(file, &waypoints)
	}

	waypoints = append(waypoints, wp)

	fileData, _ := json.Marshal(waypoints)
	os.WriteFile(waypointsFile, fileData, 0644)

	c.JSON(http.StatusOK, gin.H{
		"status":    "Saved",
		"waypoints": waypoints,
	})
}

// DeleteWaypoint removes from JSON config
func DeleteWaypoint(c *gin.Context) {
	name := c.Param("name")

	var waypoints []models.Waypoint
	if file, err := os.ReadFile(waypointsFile); err == nil {
		json.Unmarshal(file, &waypoints)
	}

	var updated []models.Waypoint
	for _, w := range waypoints {
		if w.Name != name {
			updated = append(updated, w)
		}
	}

	fileData, _ := json.Marshal(updated)
	os.WriteFile(waypointsFile, fileData, 0644)

	c.JSON(http.StatusOK, gin.H{
		"status":    "Deleted",
		"waypoints": updated,
	})
}
