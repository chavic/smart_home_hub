import { z } from "zod";

export const envSchema = z.object({
  v:  z.literal(1),
  t:  z.number().nonnegative(),
  c:  z.string().min(3).max(3),
  e:  z.string(),
  i:  z.number().int().nonnegative(),
  d:  z.record(z.any()),
});

export type Envelope = z.infer<typeof envSchema>;
