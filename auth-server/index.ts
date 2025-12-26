import { Hono } from 'hono';
import { serve } from '@hono/node-server';
import { cors } from 'hono/cors';
import { auth } from './lib/auth';
import dotenv from 'dotenv';

dotenv.config();

const app = new Hono();

app.use('/*', cors({
  origin: 'http://localhost:3000',
  credentials: true,
  allowMethods: ['POST', 'GET', 'OPTIONS'],
  allowHeaders: ['Content-Type', 'Authorization'],
}));

app.on(['POST', 'GET'], '/api/auth/**', (c) => {
    return auth.handler(c.req.raw);
});

const port = 4000;
console.log(`Auth Server running on port ${port}`);

serve({
  fetch: app.fetch,
  port
});