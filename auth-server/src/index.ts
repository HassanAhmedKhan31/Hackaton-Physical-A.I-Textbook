import "dotenv/config";
import express from "express";
import cors from "cors";
import { auth } from "./auth"; // Ensure this matches your file structure
import { toNodeHandler } from "better-auth/node";

const app = express();
const PORT = 4000;

// 1. Fix CORS (Crucial for Frontend communication)
app.use(cors({
    origin: "http://localhost:3000", // 👈 ALLOW YOUR FRONTEND
    credentials: true, // 👈 ALLOW COOKIES/SESSIONS
    methods: ["GET", "POST", "PUT", "DELETE"],
    allowedHeaders: ["Content-Type", "Authorization"],
}));

// 2. Add JSON Parsing
app.use(express.json());

// 3. Mount the Better-Auth Handler
app.all("/api/auth/*", toNodeHandler(auth));

// 4. Test Route (To check if server is alive)
app.get("/", (req, res) => {
    res.send("Auth Server is Running! 🚀");
});

app.listen(PORT, () => {
    console.log(`✅ Auth Server running on http://localhost:${PORT}`);
});