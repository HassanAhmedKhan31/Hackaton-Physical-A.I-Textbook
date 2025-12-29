import { createClient } from '@supabase/supabase-js';

// 1. Placeholder keys (Replace these with real ones later if you want login to work)
const SUPABASE_URL = 'https://placeholder.supabase.co'; 
const SUPABASE_ANON_KEY = 'placeholder-key';

// 2. Export ONLY the Supabase client
export const supabase = createClient(SUPABASE_URL, SUPABASE_ANON_KEY);

// ❌ I removed 'authClient' because you don't have the library for it yet.