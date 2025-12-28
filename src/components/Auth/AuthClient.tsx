import { createAuthClient } from "better-auth/react";

export const authClient = createAuthClient({
    // Uses Vercel URL if available, otherwise defaults to localhost
    baseURL: process.env.NEXT_PUBLIC_AUTH_URL || "http://localhost:4000"
});