package btools.web;

import java.io.IOException;

import org.teavm.jso.JSBody;
import org.teavm.jso.typedarrays.Int8Array;
import org.teavm.runtime.fs.VirtualFile;
import org.teavm.runtime.fs.VirtualFileAccessor;
import org.teavm.runtime.fs.VirtualFileSystem;

/**
 * Read-only file system for the browser build: every file read goes to {@code globalThis.brouterFs}
 * ({@code size(path)} → bytes, -1 missing, -2 directory; {@code read(path, offset, length)} → {@code Int8Array}). The host decides
 * where bytes come from (Node {@code fs}, HTTP Range requests, OPFS). The engine keeps using
 * {@code java.io.RandomAccessFile} unchanged.
 */
final class JsFileSystem implements VirtualFileSystem {
  @JSBody(params = "path", script = "return globalThis.brouterFs.size(path);")
  static native int size(String path);

  @JSBody(params = {"path", "offset", "length"}, script = "return globalThis.brouterFs.read(path, offset, length);")
  static native Int8Array read(String path, int offset, int length);

  @JSBody(params = "path", script = "return globalThis.brouterFs.list ? globalThis.brouterFs.list(path) : [];")
  static native String[] list(String path);

  @Override
  public String getUserDir() {
    return "/";
  }

  @Override
  public VirtualFile getFile(String path) {
    return new JsFile(path);
  }

  @Override
  public boolean isWindows() {
    return false;
  }

  @Override
  public String canonicalize(String path) {
    return path;
  }

  @Override
  public String[] getRoots() {
    return new String[] {"/"};
  }

  private static final class JsFile implements VirtualFile {
    private final String path;

    JsFile(String path) {
      this.path = path;
    }

    @Override
    public String getName() {
      int slash = path.lastIndexOf('/');
      return slash < 0 ? path : path.substring(slash + 1);
    }

    @Override
    public boolean isDirectory() {
      return size(path) == -2;
    }

    @Override
    public boolean isFile() {
      return size(path) >= 0;
    }

    @Override
    public String[] listFiles() {
      return list(path);
    }

    @Override
    public VirtualFileAccessor createAccessor(boolean readable, boolean writable, boolean append) {
      if (writable || append || size(path) < 0) {
        return null;
      }
      return new Accessor(path, size(path));
    }

    @Override
    public boolean createFile(String name) {
      return false;
    }

    @Override
    public boolean createDirectory(String name) {
      return false;
    }

    @Override
    public boolean delete() {
      return false;
    }

    @Override
    public boolean adopt(VirtualFile file, String name) {
      return false;
    }

    @Override
    public boolean canRead() {
      return isFile();
    }

    @Override
    public boolean canWrite() {
      return false;
    }

    @Override
    public long lastModified() {
      return 0;
    }

    @Override
    public boolean setLastModified(long lastModified) {
      return false;
    }

    @Override
    public boolean setReadOnly(boolean readOnly) {
      return false;
    }

    @Override
    public int length() {
      return Math.max(0, size(path));
    }
  }

  private static final class Accessor implements VirtualFileAccessor {
    private final String path;
    private final int size;
    private int position;

    Accessor(String path, int size) {
      this.path = path;
      this.size = size;
    }

    @Override
    public int read(byte[] buffer, int offset, int length) throws IOException {
      if (position >= size) {
        return -1;
      }
      int count = Math.min(length, size - position);
      // One copy across the JS boundary; works on both the JS and the Wasm GC backend.
      byte[] chunk = JsFileSystem.read(path, position, count).copyToJavaArray();
      int copied = chunk.length;
      System.arraycopy(chunk, 0, buffer, offset, copied);
      position += copied;
      return copied;
    }

    @Override
    public void write(byte[] buffer, int offset, int length) throws IOException {
      throw new IOException("read-only");
    }

    @Override
    public int tell() {
      return position;
    }

    @Override
    public void seek(int target) {
      position = target;
    }

    @Override
    public void skip(int amount) {
      position += amount;
    }

    @Override
    public int size() {
      return size;
    }

    @Override
    public void resize(int newSize) throws IOException {
      throw new IOException("read-only");
    }

    @Override
    public void close() {
    }

    @Override
    public void flush() {
    }
  }
}
