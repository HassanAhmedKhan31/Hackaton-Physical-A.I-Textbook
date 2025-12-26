import { betterAuth } from "better-auth";
import { Pool } from "pg";
import dotenv from "dotenv";

dotenv.config();

if (!process.env.DATABASE_URL) {
  throw new Error("DATABASE_URL is not defined");
}

const pool = new Pool({
  connectionString: process.env.DATABASE_URL,
});

export const auth = betterAuth({
  database: pool,
  emailAndPassword: {
    enabled: true,
  },
  user: {
    additionalFields: {
      background: {
        type: "string",
        required: true,
        defaultValue: "student",
        input: true 
      },
      hardware_specs: {
        type: "string",
        required: false,
        input: true
      }
    }
  },
  trustedOrigins: ["http://localhost:3000"]
});