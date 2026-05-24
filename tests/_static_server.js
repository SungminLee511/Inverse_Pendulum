// Tiny static file server used by UI tests. Default port 4173.
// Usage:
//   import { startServer } from './_static_server.js';
//   const srv = await startServer({ root: '..', port: 0 });  // 0 = random
//   ... await srv.close();

import http from 'node:http';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const __filename = fileURLToPath(import.meta.url);
const __dirname  = path.dirname(__filename);

const MIME = {
  '.html': 'text/html; charset=utf-8',
  '.js':   'text/javascript; charset=utf-8',
  '.mjs':  'text/javascript; charset=utf-8',
  '.css':  'text/css; charset=utf-8',
  '.json': 'application/json; charset=utf-8',
  '.png':  'image/png',
  '.svg':  'image/svg+xml',
  '.ico':  'image/x-icon',
  '.map':  'application/json',
};

export function startServer({ root = path.resolve(__dirname, '..'), port = 0 } = {}) {
  return new Promise((resolve, reject) => {
    const server = http.createServer((req, res) => {
      try {
        let url = decodeURIComponent(req.url.split('?')[0]);
        if (url === '/' || url === '') url = '/index.html';
        const fp = path.normalize(path.join(root, url));
        if (!fp.startsWith(root)) {
          res.writeHead(403); res.end('forbidden'); return;
        }
        fs.stat(fp, (err, st) => {
          if (err || !st.isFile()) {
            res.writeHead(404, { 'content-type': 'text/plain' });
            res.end(`not found: ${url}`);
            return;
          }
          const ext = path.extname(fp).toLowerCase();
          res.writeHead(200, {
            'content-type': MIME[ext] || 'application/octet-stream',
            'cache-control': 'no-cache',
          });
          fs.createReadStream(fp).pipe(res);
        });
      } catch (e) {
        res.writeHead(500); res.end(String(e));
      }
    });
    server.listen(port, '127.0.0.1', () => {
      const { port: p } = server.address();
      resolve({
        port: p,
        url: `http://127.0.0.1:${p}`,
        close: () => new Promise(r => server.close(r)),
      });
    });
    server.on('error', reject);
  });
}

// If run directly: `npm run serve`
if (import.meta.url === `file://${process.argv[1]}`) {
  const port = Number(process.env.PORT) || 4173;
  startServer({ port }).then(s => {
    console.log(`Serving root at ${s.url}`);
  });
}
